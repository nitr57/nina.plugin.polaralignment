using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

// General Information about an assembly is controlled through the following
// set of attributes. Change these attribute values to modify the information
// associated with an assembly.
//[assembly: AssemblyTitle("Three Point Polar Alignment")]
//[assembly: AssemblyDescription("Three Point Polar Alignment almost anywhere in the sky")]
//[assembly: AssemblyConfiguration("")]
//[assembly: AssemblyCompany("Stefan Berg")]
//[assembly: AssemblyProduct("NINA.Plugins")]
//[assembly: AssemblyCopyright("Copyright ©  2021-2025")]
[assembly: AssemblyTrademark("")]
[assembly: AssemblyCulture("")]

// Setting ComVisible to false makes the types in this assembly not visible
// to COM components.  If you need to access a type in this assembly from
// COM, set the ComVisible attribute to true on that type.
[assembly: ComVisible(false)]

// The following GUID is for the ID of the typelib if this project is exposed to COM
[assembly: Guid("1de8d7d3-f11e-494c-a371-95cb48dffa18")]

//The assembly versioning
//Should be incremented for each new release build of a plugin
//[assembly: AssemblyVersion("2.2.3.4")]
//[assembly: AssemblyFileVersion("2.2.3.4")]

//The minimum Version of N.I.N.A. that this plugin is compatible with
[assembly: AssemblyMetadata("MinimumApplicationVersion", "3.1.2.9001")]

//Your plugin homepage - omit if not applicaple
[assembly: AssemblyMetadata("Homepage", "https://www.patreon.com/stefanberg/")]
//The license your plugin code is using
[assembly: AssemblyMetadata("License", "MPL-2.0")]
//The url to the license
[assembly: AssemblyMetadata("LicenseURL", "https://www.mozilla.org/en-US/MPL/2.0/")]
//The repository where your pluggin is hosted
[assembly: AssemblyMetadata("Repository", "https://github.com/isbeorn/nina.plugin.polaralignment")]
[assembly: InternalsVisibleTo("NINA.Plugins.PolarAlignment.Test")]

[assembly: AssemblyMetadata("ChangelogURL", "https://github.com/isbeorn/nina.plugin.polaralignment/blob/master/PolarAlignment/Changelog.md")]

//Common tags that quickly describe your plugin
[assembly: AssemblyMetadata("Tags", "Polar alignment,Sequencer")]

//The featured logo that will be displayed in the plugin list next to the name
[assembly: AssemblyMetadata("FeaturedImageURL", "https://github.com/isbeorn/nina.plugin.polaralignment/blob/master/PolarAlignment/logo.png?raw=true")]
//An example screenshot of your plugin in action
[assembly: AssemblyMetadata("ScreenshotURL", "https://github.com/isbeorn/nina.plugin.polaralignment/blob/master/PolarAlignment/Starlock2.png?raw=true")]
//An additional example screenshot of your plugin in action
[assembly: AssemblyMetadata("AltScreenshotURL", "https://github.com/isbeorn/nina.plugin.polaralignment/blob/master/PolarAlignment/Imaging.png?raw=true")]
[assembly: AssemblyMetadata("LongDescription", @"Three Point Polar Alignment almost anywhere in the sky  

This plugin adds a Three Point Polar Alignment instruction to the Advanced Sequencer and a matching tool pane to the Imaging tab. The sequencer path opens a guided window during execution, and the imaging-tab path exposes the same workflow directly from the dockable tool.  

TPPA first captures and plate-solves three measurement points while the mount moves only along the right ascension axis. From those three solved positions it reconstructs the mount axis and reports the initial polar alignment error. After that it enters a continuous solve loop so you can adjust the mount altitude and azimuth while TPPA keeps updating the remaining error.  

If a supported adjustment system is selected, TPPA can connect to it after the initial three-point solve and optionally drive automated correction nudges.  

[*Frequently Asked Questions*](https://github.com/isbeorn/nina.plugin.polaralignment/blob/master/PolarAlignment/FAQ.md)

*Prerequisites*  
* Site latitude, longitude, and elevation must be set correctly in the active N.I.N.A. profile
* A connected camera that can capture frames for plate solving
* Plate solving must be configured and working with the current optical setup
    + When using manual mode without a mount connection, TPPA uses the blind solver path
* An equatorial mount whose right ascension axis can be moved using one of three methods:
    + Fully automated, with the mount connected through N.I.N.A. so TPPA can move it
    + Manual mode with the mount connected, where you move the mount through its controls or software
    + Manual mode without a mount connection, where you move the mount yourself

For best results, keep tracking enabled during the run, move only the RA axis between the first three points, and then adjust only mount altitude and azimuth during the correction phase.")]
