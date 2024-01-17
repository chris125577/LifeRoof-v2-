// TODO fill in this information for your driver, then remove this line!
//
// ASCOM Dome hardware class for LifeRoof
//
// Description:	 Replacement driver using .exe file, rather than DLL. Implements slewing property for shutter moving
//
// Implements:	ASCOM Dome interface version: <To be completed by driver developer>
// Author:		(CJW) cwoodhou@icloud.com
//

using ASCOM;
using ASCOM.Astrometry;
using ASCOM.Astrometry.AstroUtils;
using ASCOM.Astrometry.NOVAS;
using ASCOM.DeviceInterface;
using ASCOM.Utilities;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace ASCOM.LifeRoof.Dome
{
    
    /// <summary>
    /// ASCOM Dome hardware class for LifeRoof.
    /// </summary>
    [HardwareClass()] // Class attribute flag this as a device hardware class that needs to be disposed by the local server when it exits.
    internal static class DomeHardware
    {
        // Constants used for Profile persistence
        internal const string comPortProfileName = "COM Port";
        internal const string comPortDefault = "COM1";
        internal const string traceStateProfileName = "Trace Level";
        internal const string traceStateDefault = "true";

        private static string DriverProgId = ""; // ASCOM DeviceID (COM ProgID) for this driver, the value is set by the driver's class initialiser.
        private static string DriverDescription = ""; // The value is set by the driver's class initialiser.
        internal static string comPort; // COM port name (if required)
        //internal static bool traceState;  // used to hold setup log file option status
        private static string buffer = "";
        private static bool startCharRx;  // indicates start of message detected
        private static char endChar = '#';  // end character of Arduino response
        private static char startChar = '$';  // start character of Arduino response
        private static char[] delimeter = { ',' };
        private static string arduinoMessage = ""; // string that builds up received message
        private static bool dataRx = false;  // indicates that data is available to read
        private static string oldShutterStatus = "1"; // default is closed
        private static SerialPort Serial;  // my serial port instance of ASCOM serial port
        private static bool connectedState; // Local server's connected state
        private static string[] arduinoStatus = new string[6];  //  "$dry,roof,park,rainsense,parksense,beepstatus#"
        private static bool runOnce = false; // Flag to enable "one-off" activities only to run once.
        internal static Util utilities; // ASCOM Utilities object for use as required
        internal static AstroUtils astroUtilities; // ASCOM AstroUtilities object for use as required
        internal static TraceLogger tl; // Local server's trace logger object for diagnostic log with information that you specify

        /// <summary>
        /// Initializes a new instance of the device Hardware class.
        /// </summary>
        static DomeHardware()
        {
            try
            {
                // Create the hardware trace logger in the static initialiser.
                // All other initialisation should go in the InitialiseHardware method.
                tl = new TraceLogger("", "LifeRoof.Hardware");

                // DriverProgId has to be set here because it used by ReadProfile to get the TraceState flag.
                DriverProgId = Dome.DriverProgId; // Get this device's ProgID so that it can be used to read the Profile configuration values

                // ReadProfile has to go here before anything is written to the log because it loads the TraceLogger enable / disable state.
                ReadProfile(); // Read device configuration from the ASCOM Profile store, including the trace state
                connectedState = false; // Initialise connected to false
                Serial = new SerialPort();  // standard .net serial port
                LogMessage("DomeHardware", $"Static initialiser completed.");
            }
            catch (Exception ex)
            {
                try { LogMessage("DomeHardware", $"Initialisation exception: {ex}"); } catch { }
                MessageBox.Show($"{ex.Message}", "Exception creating ASCOM.LifeRoof.Dome", MessageBoxButtons.OK, MessageBoxIcon.Error);
                throw;
            }
        }

        /// <summary>
        /// Place device initialisation code here that delivers the selected ASCOM <see cref="Devices."/>
        /// </summary>
        /// <remarks>Called every time a new instance of the driver is created.</remarks>
        internal static void InitialiseHardware()
        {
            // This method will be called every time a new ASCOM client loads your driver
            LogMessage("InitialiseHardware", $"Start.");

            // Make sure that "one off" activities are only undertaken once
            if (runOnce == false)
            {
                LogMessage("InitialiseHardware", $"Starting one-off initialisation.");

                DriverDescription = Dome.DriverDescription; // Get this device's Chooser description

                LogMessage("InitialiseHardware", $"ProgID: {DriverProgId}, Description: {DriverDescription}");

                connectedState = false; // Initialise connected to false
                utilities = new Util(); //Initialise ASCOM Utilities object
                astroUtilities = new AstroUtils(); // Initialise ASCOM Astronomy Utilities object

                LogMessage("InitialiseHardware", "Completed basic initialisation");

                OpenArduino();

                LogMessage("InitialiseHardware", $"One-off initialisation complete.");
                runOnce = true; // Set the flag to ensure that this code is not run again
            }
        }
        // OpenArduino initilises serial port and set up an event handler to suck in characters
        // it runs in the background, Arduino broadcasts every 4 seconds
        static private bool OpenArduino()
        {
            Serial.BaudRate = 19200;  // note original was 9600
            Serial.PortName = comPort;
            Serial.Parity = Parity.None;
            Serial.DataBits = 8;
            Serial.Handshake = System.IO.Ports.Handshake.None;
            Serial.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(ReceiveData);
            Serial.ReceivedBytesThreshold = 1;
            try
            {
                Serial.Open();              // open port
                Serial.DiscardInBuffer();   // and clear it out just in case
            }
            catch (Exception)
            {
                return false;
            }
            return true;
        }


        // ReceiveData is based on a code fragment suggested by Per and reads characters as they arrive
        // it decodes the messages, looking for framing characters and then splits the CSV string into
        // component parts to represent the status flags from the Arduino 
        static private void ReceiveData(object sender, SerialDataReceivedEventArgs e)
        {
            if (e.EventType == System.IO.Ports.SerialData.Chars)
            {
                while (Serial.BytesToRead > 0)
                {
                    char c = (char)Serial.ReadChar();  // wait for start character
                    if (!startCharRx)
                    {
                        if (c == startChar)  // and then initialise the message
                        {
                            startCharRx = true;
                            buffer = "";  // clear buffer
                        }
                    }
                    else
                    {
                        if (c == endChar)
                        {
                            arduinoMessage = buffer;  // transfer the buffer to the message and clear the buffer
                            buffer = "";
                            startCharRx = false;
                            if (arduinoMessage.Length == 11) // check the message length is OK (was 14 with old 'spare' message)
                            {
                                dataRx = true; // tell the world that data is available
                                arduinoStatus = arduinoMessage.Split(delimeter);  // rain / roof / park / rainsense / mountsense / beep
                            }
                            else  // message was corrupted
                            {
                                dataRx = false;
                                tl.LogMessage("communications", "corrupted message length");
                                arduinoMessage = "";
                            }
                        }
                        else
                        {
                            buffer += c;  // build up message string in buffer
                        }
                    }
                }
            }
        }
        // PUBLIC COM INTERFACE IDomeV2 IMPLEMENTATION

        #region Common properties and methods.

        /// <summary>
        /// Displays the Setup Dialogue form.
        /// If the user clicks the OK button to dismiss the form, then
        /// the new settings are saved, otherwise the old values are reloaded.
        /// THIS IS THE ONLY PLACE WHERE SHOWING USER INTERFACE IS ALLOWED!
        /// </summary>
        public static void SetupDialog()
        {
            // Don't permit the setup dialogue if already connected
            if (IsConnected)
                MessageBox.Show("Already connected, just press OK");

            using (SetupDialogForm F = new SetupDialogForm(tl))
            {
                var result = F.ShowDialog();
                if (result == DialogResult.OK)
                {
                    WriteProfile(); // Persist device configuration values to the ASCOM Profile store
                }
            }
        }

        /// <summary>Returns the list of custom action names supported by this driver.</summary>
        /// <value>An ArrayList of strings (SafeArray collection) containing the names of supported actions.</value>
        public static ArrayList SupportedActions
        {
            get
            {
                ArrayList suptaction = new ArrayList()
                { "INIT","FORCEOPEN","FORCECLOSE","NORAINSENSE","NOPARKSENSE","RAINSENSE","PARKSENSE","PARKSENSOR","RAINSENSOR","BEEPON","BEEPOFF","BEEPSTATUS"};
                return suptaction;
            }
        }

        /// <summary>Invokes the specified device-specific custom action.</summary>
        /// <param name="ActionName">A well known name agreed by interested parties that represents the action to be carried out.</param>
        /// <param name="ActionParameters">List of required parameters or an <see cref="String.Empty">Empty String</see> if none are required.</param>
        /// <returns>A string response. The meaning of returned strings is set by the driver author.
        /// <para>Suppose filter wheels start to appear with automatic wheel changers; new actions could be <c>QueryWheels</c> and <c>SelectWheel</c>. The former returning a formatted list
        /// of wheel names and the second taking a wheel name and making the change, returning appropriate values to indicate success or failure.</para>
        /// </returns>
        public static string Action(string actionName, string actionParameters)
        {
            LogMessage("Action", $"Action {actionName}, parameters {actionParameters} is not implemented");
            throw new ActionNotImplementedException("Action " + actionName + " is not implemented by this driver");
        }

        /// <summary>
        /// Transmits an arbitrary string to the device and does not wait for a response.
        /// Optionally, protocol framing characters may be added to the string before transmission.
        /// </summary>
        /// <param name="Command">The literal command string to be transmitted.</param>
        /// <param name="Raw">
        /// if set to <c>true</c> the string is transmitted 'as-is'.
        /// If set to <c>false</c> then protocol framing characters may be added prior to transmission.
        /// </param>
        public static void CommandBlind(string command, bool raw)
        {
            CheckConnected("CommandBlind");
            CommandString(command, raw);  // don't need the return string
            return;
        }

        /// <summary>
        /// Transmits an arbitrary string to the device and waits for a boolean response.
        /// Optionally, protocol framing characters may be added to the string before transmission.
        /// </summary>
        /// <param name="Command">The literal command string to be transmitted.</param>
        /// <param name="Raw">
        /// if set to <c>true</c> the string is transmitted 'as-is'.
        /// If set to <c>false</c> then protocol framing characters may be added prior to transmission.
        /// </param>
        /// <returns>
        /// Returns the interpreted boolean response received from the device.
        /// </returns>
        public static bool CommandBool(string command, bool raw)
        {
            CheckConnected("CommandBool");
            return (CommandString(command, raw) == "1");// return state of command from arduino
        }

        /// <summary>
        /// Transmits an arbitrary string to the device and waits for a string response.
        /// Optionally, protocol framing characters may be added to the string before transmission.
        /// </summary>
        /// <param name="Command">The literal command string to be transmitted.</param>
        /// <param name="Raw">
        /// if set to <c>true</c> the string is transmitted 'as-is'.
        /// If set to <c>false</c> then protocol framing characters may be added prior to transmission.
        /// </param>
        /// <returns>
        /// Returns the string response received from the device.
        /// </returns>
        public static string CommandString(string command, bool raw)
        {
            CheckConnected("CommandString");
            // this is the customised I/O to the serial port, used by all commands
            // status commands are interpreted from cache variables, and commands
            // are issued
            try
            {
                if (!raw) // if command uses delimiter
                {
                    //tl.LogMessage("attempting commandstring", command);
                    if (IsConnected)  // only if connected  - try and avoid comms error
                    {
                        if (command == "RAIN" && dataRx) return (arduinoStatus[0]);         // these return status from Arduino message array
                        else if (command == "SHUTTERSTATUS" && dataRx) return (arduinoStatus[1]);
                        else if (command == "PARK" && dataRx) return (arduinoStatus[2]);
                        else if (command == "RAINSENSOR" && dataRx) return (arduinoStatus[3]);
                        else if (command == "PARKSENSOR" && dataRx) return (arduinoStatus[4]);
                        else if (command == "BEEPSTATUS" && dataRx) return (arduinoStatus[5]);
                        else
                        {
                            if (command == "SHUTTERSTATUS") tl.LogMessage("comms error", "no shutter data");  // diagnostic
                            Serial.Write(command + "#"); return ("1");  // if not a data request, it sends the string command   (bool/blind/string) 
                        }
                    }
                    throw new ASCOM.NotConnectedException("com port not connected");
                }
                else  // do nothing if the command is not using delimiter
                {
                    tl.LogMessage("commandstring ", "Not implemented without # terminator");
                    throw new ASCOM.MethodNotImplementedException("CommandString");
                }
            }
            catch (Exception)  // better luck next time :)
            {
                System.Windows.Forms.MessageBox.Show("Timed out, press OK to recover");
                return ("comms error");
            }
        }

        /// <summary>
        /// Deterministically release both managed and unmanaged resources that are used by this class.
        /// </summary>
        /// <remarks>
        /// 
        /// 
        /// </remarks>
        public static void Dispose()
        {
            try { LogMessage("Dispose", $"Disposing of assets and closing down."); } catch { }

            try
            {
                // Clean up the trace logger and utility objects
                tl.Enabled = false;
                tl.Dispose();
                tl = null;
            }
            catch { }

            try
            {
                utilities.Dispose();
                utilities = null;
            }
            catch { }

            try
            {
                astroUtilities.Dispose();
                astroUtilities = null;
            }
            catch { }
            try
            {
                Serial.Dispose();
                Serial = null;
            }
            catch { }
        }

        /// <summary>
        /// Set True to connect to the device hardware. Set False to disconnect from the device hardware.
        /// You can also read the property to check whether it is connected. This reports the current hardware state.
        /// </summary>
        /// <value><c>true</c> if connected to the hardware; otherwise, <c>false</c>.</value>
        public static bool Connected
        {
            get
            {
                LogMessage("Connected", $"Get {IsConnected}");
                return IsConnected;
            }
            set
            {
                LogMessage("Connected", $"Set {value}");
                if (value == IsConnected)
                    return;

                if (value)
                {
                    LogMessage("Connected Set", $"Connecting to port {comPort}");
                    connectedState = true;
                }
                else
                {
                    connectedState = false;
                    LogMessage("Connected Set", "Disconnecting from port {0}", comPort);
                    Serial.Close();  //disconnect to serial
                    Serial.Dispose();
                }
            }
        }

        /// <summary>
        /// Returns a description of the device, such as manufacturer and model number. Any ASCII characters may be used.
        /// </summary>
        /// <value>The description.</value>
        public static string Description
        {
            get
            {
                LogMessage("Description Get", DriverDescription);
                return DriverDescription;
            }
        }

        /// <summary>
        /// Descriptive and version information about this ASCOM driver.
        /// </summary>
        public static string DriverInfo
        {
            get
            {
                Version version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;
                string driverInfo = $"Information about the driver itself. Version: {version.Major}.{version.Minor}";
                LogMessage("DriverInfo Get", driverInfo);
                return driverInfo;
            }
        }

        /// <summary>
        /// A string containing only the major and minor version of the driver formatted as 'm.n'.
        /// </summary>
        public static string DriverVersion
        {
            get
            {
                Version version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;
                string driverVersion = $"{version.Major}.{version.Minor}";
                LogMessage("DriverVersion Get", driverVersion);
                return driverVersion;
            }
        }

        /// <summary>
        /// The interface version number that this device supports.
        /// </summary>
        public static short InterfaceVersion
        {
            // set by the driver wizard
            get
            {
                LogMessage("InterfaceVersion Get", "2");
                return Convert.ToInt16("2");
            }
        }

        /// <summary>
        /// The short name of the driver, for display purposes
        /// </summary>
        public static string Name
        {
            get
            {
                string name = "LifeRoof";
                LogMessage("Name Get", name);
                return name;
            }
        }

        #endregion

        #region IDome Implementation

        private static bool domeShutterState = false; // Variable to hold the open/closed status of the shutter, true = Open

        /// <summary>
        /// Immediately stops any and all movement of the dome.
        /// </summary>
        internal static void AbortSlew()
        {
            // This is a mandatory parameter but we have no action to take in this simple driver
            LogMessage("AbortSlew", "Completed");
        }

        /// <summary>
        /// The altitude (degrees, horizon zero and increasing positive to 90 zenith) of the part of the sky that the observer wishes to observe.
        /// </summary>
        internal static double Altitude
        {
            get
            {
                LogMessage("Altitude Get", "Not implemented");
                throw new PropertyNotImplementedException("Altitude", false);
            }
        }

        /// <summary>
        /// <para><see langword="true" /> when the dome is in the home position. Raises an error if not supported.</para>
        /// <para>
        /// This is normally used following a <see cref="FindHome" /> operation. The value is reset
        /// with any azimuth slew operation that moves the dome away from the home position.
        /// </para>
        /// <para>
        /// <see cref="AtHome" /> may optionally also become true during normal slew operations, if the
        /// dome passes through the home position and the dome controller hardware is capable of
        /// detecting that; or at the end of a slew operation if the dome comes to rest at the home
        /// position.
        /// </para>
        /// </summary>
        internal static bool AtHome
        {
            get
            {
                LogMessage("AtHome Get", "Not implemented");
                throw new PropertyNotImplementedException("AtHome", false);
            }
        }

        /// <summary>
        /// <see langword="true" /> if the dome is in the programmed park position.
        /// </summary>
        internal static bool AtPark
        {
            get
            {
                LogMessage("AtPark Get", "Not implemented");
                throw new PropertyNotImplementedException("AtPark", false);
            }
        }

        /// <summary>
        /// The dome azimuth (degrees, North zero and increasing clockwise, i.e., 90 East, 180 South, 270 West). North is true north and not magnetic north.
        /// </summary>
        internal static double Azimuth
        {
            get
            {
                LogMessage("Azimuth Get", "Not implemented");
                throw new PropertyNotImplementedException("Azimuth", false);
            }
        }

        /// <summary>
        /// <see langword="true" /> if driver can perform a search for home position.
        /// </summary>
        internal static bool CanFindHome
        {
            get
            {
                LogMessage("CanFindHome Get", false.ToString());
                return false;
            }
        }

        /// <summary>
        /// <see langword="true" /> if the driver is capable of parking the dome.
        /// </summary>
        internal static bool CanPark
        {
            get
            {
                LogMessage("CanPark Get", false.ToString());
                return false;
            }
        }

        /// <summary>
        /// <see langword="true" /> if driver is capable of setting dome altitude.
        /// </summary>
        internal static bool CanSetAltitude
        {
            get
            {
                LogMessage("CanSetAltitude Get", false.ToString());
                return false;
            }
        }

        /// <summary>
        /// <see langword="true" /> if driver is capable of rotating the dome. Muste be <see "langword="false" /> for a 
        /// roll-off roof or clamshell.
        /// </summary>
        internal static bool CanSetAzimuth
        {
            get
            {
                LogMessage("CanSetAzimuth Get", false.ToString());
                return false;
            }
        }

        /// <summary>
        /// <see langword="true" /> if the driver can set the dome park position.
        /// </summary>
        internal static bool CanSetPark
        {
            get
            {
                LogMessage("CanSetPark Get", false.ToString());
                return false;
            }
        }

        /// <summary>
        /// <see langword="true" /> if the driver is capable of opening and closing the shutter or roof
        /// mechanism.
        /// </summary>
        internal static bool CanSetShutter
        {
            get
            {
                LogMessage("CanSetShutter Get", true.ToString());
                return true;
            }
        }

        /// <summary>
        /// <see langword="true" /> if the dome hardware supports slaving to a telescope.
        /// </summary>
        internal static bool CanSlave
        {
            get
            {
                LogMessage("CanSlave Get", false.ToString());
                return false;
            }
        }

        /// <summary>
        /// <see langword="true" /> if the driver is capable of synchronizing the dome azimuth position
        /// using the <see cref="SyncToAzimuth" /> method.
        /// </summary>
        internal static bool CanSyncAzimuth
        {
            get
            {
                LogMessage("CanSyncAzimuth Get", false.ToString());
                return false;
            }
        }

        /// <summary>
        /// Close the shutter or otherwise shield the telescope from the sky.
        /// </summary>
        internal static void CloseShutter()
        {
            try
            {
                var t = Task.Run(async delegate { await Task.Delay(600); });
                dataRx = false;  // new received command will change this
                CommandBlind("CLOSE", false);  // fed through CommandBlind and then CommandString
                tl.LogMessage("CloseShutter", "Shutter asked to close");
                {
                    int counter = 0;
                    while (!dataRx || counter < 5)
                    {
                        t.Wait();
                        counter++;
                    }
                    if (!dataRx) throw new ASCOM.NotConnectedException("no reply!");
                }
                domeShutterState = false;
            }
            catch (Exception)
            {
                tl.LogMessage("Comms issue", "No response from Roof within poll time");
            }
        }

        /// <summary>
        /// Start operation to search for the dome home position.
        /// </summary>
        internal static void FindHome()
        {
            LogMessage("FindHome", "Not implemented");
            throw new MethodNotImplementedException("FindHome");
        }

        /// <summary>
        /// Open shutter or otherwise expose telescope to the sky.
        /// </summary>
        internal static void OpenShutter()
        {
            try
            {
                var t = Task.Run(async delegate { await Task.Delay(600); }); 
                dataRx = false;
                CommandBlind("OPEN", false);  // fed through CommandBlind and then CommandString
                tl.LogMessage("OpenShutter", "Shutter asked to open");
                {
                    int counter = 0;
                    while (!dataRx || counter < 5)
                    {
                        t.Wait();
                        counter++;
                    }
                    if (!dataRx) throw new ASCOM.NotConnectedException("no reply!");
                }
                domeShutterState = true; // what is this?
            }
            catch (Exception)
            {
                tl.LogMessage("Comms issue", "No response from Roof within poll time");
            }
        }

        /// <summary>
        /// Rotate dome in azimuth to park position.
        /// </summary>
        internal static void Park()
        {
            LogMessage("Park", "Not implemented");
            throw new MethodNotImplementedException("Park");
        }

        /// <summary>
        /// Set the current azimuth position of dome to the park position.
        /// </summary>
        internal static void SetPark()
        {
            LogMessage("SetPark", "Not implemented");
            throw new MethodNotImplementedException("SetPark");
        }

        /// <summary>
        /// Gets the status of the dome shutter or roof structure.
        /// </summary>
        internal static ShutterState ShutterStatus
        {
            // modified so that log file only registers changed state
            get
            {
                string status = CommandString("SHUTTERSTATUS", false);
                tl.LogMessage("SHUTTERSTATUS", status);
                switch (status)
                {
                    case "0":
                        if (oldShutterStatus != status) tl.LogMessage("ShutterStatus", ShutterState.shutterOpen.ToString());
                        oldShutterStatus = status;
                        return ShutterState.shutterOpen;
                    case "1":
                        if (oldShutterStatus != status) tl.LogMessage("ShutterStatus", ShutterState.shutterClosed.ToString());
                        oldShutterStatus = status;
                        return ShutterState.shutterClosed;
                    case "2":
                        if (oldShutterStatus != status) tl.LogMessage("ShutterStatus", ShutterState.shutterOpening.ToString());
                        oldShutterStatus = status;
                        return ShutterState.shutterOpening;
                    case "3":
                        if (oldShutterStatus != status) tl.LogMessage("ShutterStatus", ShutterState.shutterClosing.ToString());
                        oldShutterStatus = status;
                        return ShutterState.shutterClosing;
                    default:
                        if (oldShutterStatus != status) tl.LogMessage("ShutterStatus", ShutterState.shutterError.ToString());
                        oldShutterStatus = status;
                        return ShutterState.shutterError;
                }
            }
        }

        /// <summary>
        /// <see langword="true"/> if the dome is slaved to the telescope in its hardware, else <see langword="false"/>.
        /// </summary>
        internal static bool Slaved
        {
            get
            {
                LogMessage("Slaved Get", false.ToString());
                return false;
            }
            set
            {
                LogMessage("Slaved Set", "not implemented");
                throw new PropertyNotImplementedException("Slaved", true);
            }
        }

        /// <summary>
        /// Ensure that the requested viewing altitude is available for observing.
        /// </summary>
        /// <param name="Altitude">
        /// The desired viewing altitude (degrees, horizon zero and increasing positive to 90 degrees at the zenith)
        /// </param>
        internal static void SlewToAltitude(double Altitude)
        {
            LogMessage("SlewToAltitude", "Not implemented");
            throw new MethodNotImplementedException("SlewToAltitude");
        }

        /// <summary>
        /// Ensure that the requested viewing azimuth is available for observing.
        /// The method should not block and the slew operation should complete asynchronously.
        /// </summary>
        /// <param name="Azimuth">
        /// Desired viewing azimuth (degrees, North zero and increasing clockwise. i.e., 90 East,
        /// 180 South, 270 West)
        /// </param>
        internal static void SlewToAzimuth(double Azimuth)
        {
            LogMessage("SlewToAzimuth", "Not implemented");
            throw new MethodNotImplementedException("SlewToAzimuth");
        }

        /// <summary>
        /// <see langword="true" /> if any part of the dome is currently moving or a move command has been issued, 
        /// but the dome has not yet started to move. <see langword="false" /> if all dome components are stationary
        /// and no move command has been issued. /> 
        /// </summary>
        internal static bool Slewing
        {
            get
            {
                bool slew = (ShutterStatus == ShutterState.shutterOpening || ShutterStatus == ShutterState.shutterClosing);
                tl.LogMessage("Shutter moving", slew.ToString());
                return slew;
            }
        }

        /// <summary>
        /// Synchronize the current position of the dome to the given azimuth.
        /// </summary>
        /// <param name="Azimuth">
        /// Target azimuth (degrees, North zero and increasing clockwise. i.e., 90 East,
        /// 180 South, 270 West)
        /// </param>
        internal static void SyncToAzimuth(double Azimuth)
        {
            LogMessage("SyncToAzimuth", "Not implemented");
            throw new MethodNotImplementedException("SyncToAzimuth");
        }

        #endregion

        #region Private properties and methods
        // Useful methods that can be used as required to help with driver development

        /// <summary>
        /// Returns true if there is a valid connection to the driver hardware
        /// </summary>
        private static bool IsConnected
        {
            get
            {
                // check the actual serial connection (checks for unplugged)
                connectedState = Serial.IsOpen; 
                return connectedState;
            }
        }

        /// <summary>
        /// Use this function to throw an exception if we aren't connected to the hardware
        /// </summary>
        /// <param name="message"></param>
        private static void CheckConnected(string message)
        {
            if (!IsConnected)
            {
                throw new NotConnectedException(message);
            }
        }

        /// <summary>
        /// Read the device configuration from the ASCOM Profile store
        /// </summary>
        internal static void ReadProfile()
        {
            using (Profile driverProfile = new Profile())
            {
                driverProfile.DeviceType = "Dome";
                tl.Enabled = Convert.ToBoolean(driverProfile.GetValue(DriverProgId, traceStateProfileName, string.Empty, traceStateDefault));
                comPort = driverProfile.GetValue(DriverProgId, comPortProfileName, string.Empty, comPortDefault);
            }
        }

        /// <summary>
        /// Write the device configuration to the  ASCOM  Profile store
        /// </summary>
        internal static void WriteProfile()
        {
            using (Profile driverProfile = new Profile())
            {
                driverProfile.DeviceType = "Dome";
                driverProfile.WriteValue(DriverProgId, traceStateProfileName, tl.Enabled.ToString());
                driverProfile.WriteValue(DriverProgId, comPortProfileName, comPort.ToString());
            }
        }

        /// <summary>
        /// Log helper function that takes identifier and message strings
        /// </summary>
        /// <param name="identifier"></param>
        /// <param name="message"></param>
        internal static void LogMessage(string identifier, string message)
        {
            tl.LogMessageCrLf(identifier, message);
        }

        /// <summary>
        /// Log helper function that takes formatted strings and arguments
        /// </summary>
        /// <param name="identifier"></param>
        /// <param name="message"></param>
        /// <param name="args"></param>
        internal static void LogMessage(string identifier, string message, params object[] args)
        {
            var msg = string.Format(message, args);
            LogMessage(identifier, msg);
        }
        #endregion
    }
}

