/***************************************************************************
 *
 * Project:  OpenCPN
 * Purpose:  OpenCPN Main wxWidgets Program
 * Author:   David Register
 *
 ***************************************************************************
 *   Copyright (C) 2010 by David S. Register                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301,  USA.         *
 **************************************************************************/

#include "config.h"

#include <iostream>
#include <regex>
#include <string>
#include <unordered_set>
#include <vector>

#ifdef __MINGW32__
#undef IPV6STRICT    // mingw FTBS fix:  missing struct ip_mreq
#include <windows.h>
#endif

#include <wx/arrstr.h>
#include <wx/log.h>
#include <wx/utils.h>

#ifdef OCPN_USE_NEWSERIAL
#include "serial/serial.h"
#endif

#ifdef HAVE_LIBUDEV
#include "libudev.h"
#endif

#ifdef HAVE_DIRENT_H
#include "dirent.h"
#endif

#ifdef HAVE_LINUX_SERIAL_H
#include "linux/serial.h"
#endif

#ifdef HAVE_SYS_IOCTL_H
#include <sys/ioctl.h>
#endif

#ifdef HAVE_SYS_FCNTL_H
#include <sys/fcntl.h>
#endif

#ifdef HAVE_SYS_TYPES_H
#include <sys/types.h>
#endif

#ifdef HAVE_READLINK
#include <unistd.h>
#endif

#ifdef __WXMSW__
#include <windows.h>
#include <setupapi.h>
#endif

#ifdef __WXOSX__
#include "macutils.h"
#endif

#include "gui_lib.h"
#include "GarminProtocolHandler.h"


#ifdef  __WXMSW__
DEFINE_GUID( GARMIN_DETECT_GUID, 0x2c9c45c2L, 0x8e7d, 0x4c08, 0xa1, 0x2d, 0x81, 0x6b, 0xba, 0xe7,
        0x22, 0xc0 );
#endif


#ifdef __MINGW32__ // do I need this because of mingw, or because I am running mingw under wine?
# ifndef GUID_CLASS_COMPORT
DEFINE_GUID(GUID_CLASS_COMPORT, 0x86e0d1e0L, 0x8089, 0x11d0, 0x9c, 0xe4, 0x08, 0x00, 0x3e, 0x30, 0x1f, 0x73);
# endif
#endif

extern int              g_nCOMPortCheck;

struct device_data {
    std::string info;      // Free format info text, possibly empty
    std::string path;      // Complete /dev device path
    device_data(const std::string& p, const std::string& i): info(i), path(p) {}
};


struct symlink {
    std::string path;
    std::string target;
    symlink(const std::string& p, const std::string& t): path(p), target(t) {}
};


static int isTTYreal(const char *dev)
{
    int ret = 0;
#ifdef __NetBSD__
    if (strncmp("/dev/tty0", dev, 9) == 0)
	return 1;
    if (strncmp("/dev/ttyU", dev, 9) == 0)
	return 1;
    if (strcmp("/dev/gps", dev) == 0)
	return 1;
    return 0;
#elif defined(HAVE_LINUX_SERIAL_H) && defined (HAVE_SYS_STAT_H)
    struct serial_struct serinfo;

    int fd = open(dev, O_RDONLY | O_NONBLOCK | O_NOCTTY);

    // device name is pointing to a real device
    if(fd >= 0) {
        if (ioctl(fd, TIOCGSERIAL, &serinfo)==0) {
            // If device type is no PORT_UNKNOWN we accept the port
            if (serinfo.type != PORT_UNKNOWN)
                ret = 1;
        }
        close (fd);
    }
#endif /* !NetBSD */
    return ret;
}


static bool isTTYreal(const device_data& data) {
  return isTTYreal(data.path.c_str());
}


#if defined(HAVE_DIRENT_H) && defined(HAVE_READLINK)

#define HAVE_SYSFS_PORTS

/** Return list of full paths to all possible ttys. */
static std::vector<std::string> get_device_candidates()
{
    std::vector<std::string> devices;
    DIR* dir;
    struct dirent* ent;
    dir = opendir("/sys/class/tty");
    if (dir == 0) {
        wxLogWarning("Cannot open /sys/class/tty: %s", strerror(errno));
        return devices;
    }
    const std::string prefix("/dev/");
    for (ent = readdir(dir); ent; ent = readdir(dir)) {
        devices.push_back(prefix + ent->d_name);
    }
    closedir(dir);
    return devices;
}


/** Return all symlinks directly under /dev. */
static std::vector<struct symlink> get_all_links()
{
    std::vector<struct symlink> links;
    DIR* dir;
    struct dirent* ent;
    dir = opendir("/dev");
    if (dir == 0) {
        wxLogError("Cannot open /dev: %s", strerror(errno));
        return links;
    }
    const std::string prefix("/dev/");
    for (ent = readdir(dir); ent; ent = readdir(dir)) {
        struct stat buf;
        const std::string path(prefix + ent->d_name);
        int r = lstat(path.c_str(), &buf);
        if (r == -1) {
            wxLogDebug("get_all_links: Cannot stat %s: %s",
                       path.c_str(), strerror(errno));
        }    
        else if (S_ISLNK(buf.st_mode)) {
            char buff[PATH_MAX + 1];
            readlink(path.c_str(), buff, PATH_MAX);
            std::string target(buff);
            struct symlink link(path.c_str(), prefix + target);
            links.push_back(link);
        }
    }
    closedir(dir);
    return links;
}


/** Return list of full paths to active, serial ports based on /sys files */
static wxArrayString *EnumerateSysfsSerialPorts( void )
{
    std::vector<std::string> ports;
    auto all_ports = get_device_candidates();
    wxLogDebug("Enumerate: found %d candidates", all_ports.size());
    for (auto p: all_ports) {
        if (isTTYreal(p.c_str())) ports.push_back(p); 
    }
    wxLogDebug("Enumerate: found %d good ports", ports.size());
    const auto targets =
        std::unordered_set<std::string>(ports.begin(), ports.end());

    auto all_links = get_all_links();
    wxLogDebug("Enumerate: found %d links", all_links.size());
    for (auto l: all_links) {
        if (targets.find(l.target) != targets.end()) ports.push_back(l.path); 
    }
    wxLogDebug("Enumerate: found %d devices", ports.size());

    auto wx_ports = new wxArrayString();
    for (auto p: ports) {
        wx_ports->Add(p);
    }
    return wx_ports;
}

#endif  // HAVE_DIRENT_H && defined(HAVE_READLINK)


#if defined(HAVE_LIBUDEV)  

/** Return a single string of free-format device info, possibly empty. */
std::string get_device_info(struct udev_device* ud)
{
    std::string info;
    const char* prop = udev_device_get_property_value(ud, "ID_VENDOR");
    if (prop) info += prop;
    prop = udev_device_get_property_value(ud, "ID_MODEL");
    if (prop) info += std::string(" - " ) + prop;
    return info;
}


/** Return list of device links pointing to dev. */
static std::vector<struct device_data> get_links(struct udev_device* dev,
                                                 const std::regex& exclude)
{
    std::vector<struct device_data> items;
    std::string info(" link -> ");
    info += udev_device_get_devnode(dev);
    struct udev_list_entry* link = udev_device_get_devlinks_list_entry(dev);
    while (link) {
        const char* linkname = udev_list_entry_get_name(link);
        if (! std::regex_search(linkname , exclude)) {
            struct device_data item(linkname, info);
            items.push_back(item);
        }
        link = udev_list_entry_get_next(link);
    }
    return items;
}


static std::vector<struct device_data> enumerate_udev_ports(struct udev* udev)
{
    struct udev_enumerate* enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "tty");
    udev_enumerate_scan_devices(enumerate);
    struct udev_list_entry *devices = udev_enumerate_get_list_entry(enumerate);

    const std::regex bad_ttys(".*tty[0-9][0-9]|^/dev/serial/.*");
    std::vector<struct device_data> items;
    struct udev_list_entry *entry;
    udev_list_entry_foreach(entry, devices) {
        const char* const path = udev_list_entry_get_name(entry);
        struct udev_device* device = udev_device_new_from_syspath(udev, path);
        const char* const devnode = udev_device_get_devnode(device);
        if (!std::regex_search(devnode , bad_ttys) && isTTYreal(devnode)) {
            struct device_data item(devnode, get_device_info(device));
            items.push_back(item);
            auto links = get_links(device, bad_ttys);
            items.insert(items.end(), links.begin(), links.end());
        }
        udev_device_unref(device);
    }
    return items;
}


wxArrayString *EnumerateUdevSerialPorts( void )
{
    struct udev* udev = udev_new();
    auto dev_items = enumerate_udev_ports(udev);
    wxArrayString *ports = new wxArrayString;
    for (auto item: dev_items) {
        ports->Add((item.path + " - " + item.info).c_str());
    }
    return ports;
}

#endif  // HAVE_LIBUDEV


#ifdef __WXMSW__
wxArrayString *EnumerateWindowsSerialPorts( void )
{
    wxArrayString *preturn = new wxArrayString;
    /*************************************************************************
     * Windows provides no system level enumeration of available serial ports
     * There are several ways of doing this.
     *
     *************************************************************************/

    //    Method 1:  Use GetDefaultCommConfig()
    // Try first {g_nCOMPortCheck} possible COM ports, check for a default configuration
    //  This method will not find some Bluetooth SPP ports
    for( int i = 1; i < g_nCOMPortCheck; i++ ) {
        wxString s;
        s.Printf( _T("COM%d"), i );

        COMMCONFIG cc;
        DWORD dwSize = sizeof(COMMCONFIG);
        if( GetDefaultCommConfig( s.fn_str(), &cc, &dwSize ) )
            preturn->Add( wxString( s ) );
    }

#if 0
    // Method 2:  Use FileOpen()
    // Try all 255 possible COM ports, check to see if it can be opened, or if
    // not, that an expected error is returned.

    BOOL bFound;
    for (int j=1; j<256; j++)
    {
        char s[20];
        sprintf(s, "\\\\.\\COM%d", j);

        // Open the port tentatively
        BOOL bSuccess = FALSE;
        HANDLE hComm = ::CreateFile(s, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);

        //  Check for the error returns that indicate a port is there, but not currently useable
        if (hComm == INVALID_HANDLE_VALUE)
        {
            DWORD dwError = GetLastError();

            if (dwError == ERROR_ACCESS_DENIED ||
                    dwError == ERROR_GEN_FAILURE ||
                    dwError == ERROR_SHARING_VIOLATION ||
                    dwError == ERROR_SEM_TIMEOUT)
            bFound = TRUE;
        }
        else
        {
            bFound = TRUE;
            CloseHandle(hComm);
        }

        if (bFound)
        preturn->Add(wxString(s));
    }
#endif  // 0

    // Method 3:  WDM-Setupapi
    //  This method may not find XPort virtual ports,
    //  but does find Bluetooth SPP ports

    GUID *guidDev = (GUID*) &GUID_CLASS_COMPORT;

    HDEVINFO hDevInfo = INVALID_HANDLE_VALUE;

    hDevInfo = SetupDiGetClassDevs( guidDev,
                                     NULL,
                                     NULL,
                                     DIGCF_PRESENT | DIGCF_DEVICEINTERFACE );

    if(hDevInfo != INVALID_HANDLE_VALUE) {

        BOOL bOk = TRUE;
        SP_DEVICE_INTERFACE_DATA ifcData;

        ifcData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
        for (DWORD ii=0; bOk; ii++) {
            bOk = SetupDiEnumDeviceInterfaces(hDevInfo, NULL, guidDev, ii, &ifcData);
            if (bOk) {
            // Got a device. Get the details.

                SP_DEVINFO_DATA devdata = {sizeof(SP_DEVINFO_DATA)};
                bOk = SetupDiGetDeviceInterfaceDetail(hDevInfo,
                                                      &ifcData, NULL, 0, NULL, &devdata);

                //      We really only need devdata
                if( !bOk ) {
                    if( GetLastError() == 122)  //ERROR_INSUFFICIENT_BUFFER, OK in this case
                        bOk = true;
                }

                //      We could get friendly name and/or description here
                TCHAR fname[256] = {0};
                TCHAR desc[256] ={0};
                if (bOk) {
                    BOOL bSuccess = SetupDiGetDeviceRegistryProperty(
                        hDevInfo, &devdata, SPDRP_FRIENDLYNAME, NULL,
                        (PBYTE)fname, sizeof(fname), NULL);

                    bSuccess = bSuccess && SetupDiGetDeviceRegistryProperty(
                        hDevInfo, &devdata, SPDRP_DEVICEDESC, NULL,
                        (PBYTE)desc, sizeof(desc), NULL);
                }

                //  Get the "COMn string from the registry key
                if(bOk) {
                    bool bFoundCom = false;
                    TCHAR dname[256];
                    HKEY hDeviceRegistryKey = SetupDiOpenDevRegKey(hDevInfo, &devdata,
                                                                   DICS_FLAG_GLOBAL, 0,
                                                                   DIREG_DEV, KEY_QUERY_VALUE);
                    if(INVALID_HANDLE_VALUE != hDeviceRegistryKey) {
                            DWORD RegKeyType;
                            wchar_t    wport[80];
                            LPCWSTR cstr = wport;
                            MultiByteToWideChar( 0, 0, "PortName", -1, wport, 80);
                            DWORD len = sizeof(dname);

                            int result = RegQueryValueEx(hDeviceRegistryKey, cstr,
                                                        0, &RegKeyType, (PBYTE)dname, &len );
                            if( result == 0 )
                                bFoundCom = true;
                    }

                    if( bFoundCom ) {
                        wxString port( dname, wxConvUTF8 );

                        //      If the port has already been found, remove the prior entry
                        //      in favor of this entry, which will have descriptive information appended
                        for( unsigned int n=0 ; n < preturn->GetCount() ; n++ ) {
                            if((preturn->Item(n)).IsSameAs(port)){
                                preturn->RemoveAt( n );
                                break;
                            }
                        }
                        wxString desc_name( desc, wxConvUTF8 );         // append "description"
                        port += _T(" ");
                        port += desc_name;

                        preturn->Add( port );
                    }
                }
            }
        } //for
    } // if


    //  Search for Garmin device driver on Windows platforms

    HDEVINFO hdeviceinfo = INVALID_HANDLE_VALUE;

    hdeviceinfo = SetupDiGetClassDevs( (GUID *) &GARMIN_DETECT_GUID, NULL, NULL,
            DIGCF_PRESENT | DIGCF_INTERFACEDEVICE );

    if( hdeviceinfo != INVALID_HANDLE_VALUE ) {

        if(GarminProtocolHandler::IsGarminPlugged()){
            wxLogMessage( _T("EnumerateSerialPorts() Found Garmin USB Device.") );
            preturn->Add( _T("Garmin-USB") );         // Add generic Garmin selectable device
        }
    }

#if 0
    SP_DEVICE_INTERFACE_DATA deviceinterface;
    deviceinterface.cbSize = sizeof(deviceinterface);

    if (SetupDiEnumDeviceInterfaces(hdeviceinfo,
                    NULL,
                    (GUID *) &GARMIN_DETECT_GUID,
                    0,
                    &deviceinterface))
    {
        wxLogMessage(_T("Found Garmin Device."));

        preturn->Add(_T("GARMIN"));         // Add generic Garmin selectable device
    }
#endif   // 0
    return preturn;
}

#endif

#if defined(OCPN_USE_SYSFS_PORTS) && defined(HAVE_SYSFS_PORTS)

wxArrayString *EnumerateSerialPorts( void )
{
    return EnumerateSysfsSerialPorts();
}


#elif defined(OCPN_USE_UDEV_PORTS) && defined(HAVE_LIBUDEV)

wxArrayString *EnumerateSerialPorts( void )
{
    return EnumerateUdevSerialPorts();
}


#elif defined(__OCPN__ANDROID__)

wxArrayString *EnumerateSerialPorts( void )
{
    return androidGetSerialPortsArray();
}


#elif defined(__WSOSX__)

wxArrayString *EnumerateSerialPorts( void )
{
    wxArrayString *preturn = new wxArrayString;
    char* paPortNames[MAX_SERIAL_PORTS];
    int iPortNameCount;

    memset(paPortNames,0x00,sizeof(paPortNames));
    iPortNameCount = FindSerialPortNames(&paPortNames[0],MAX_SERIAL_PORTS);
    for (int iPortIndex=0; iPortIndex<iPortNameCount; iPortIndex++)
    {
        wxString sm(paPortNames[iPortIndex], wxConvUTF8);
        preturn->Add(sm);
        free(paPortNames[iPortIndex]);
    }
    return preturn;
}


#elif defined(__WXMSW__)

wxArrayString *EnumerateSerialPorts( void )
{
    return EnumerateWindowsSerialPorts();
}

#elif !defined(__WXMSW__)

wxArrayString *EnumerateSerialPorts( void )
{
    wxArrayString *preturn = new wxArrayString;
#ifdef OCPN_USE_NEWSERIAL
    std::vector<serial::PortInfo> ports = serial::list_ports();
    for(auto it = ports.begin(); it != ports.end(); ++it) {
        wxString port((*it).port);
        if( (*it).description.length() > 0 && (*it).description != "n/a" ) {
            port.Append(_T(" - "));
            wxString s_description = wxString::FromUTF8( ((*it).description).c_str());
            port.Append( s_description );
        }
        preturn->Add(port);
    }

#else  // OPCN_USE_NEWSERIAL

#if defined(__UNIX__)

    //Initialize the pattern table
    if( devPatern[0] == NULL ) {
        paternAdd ( "ttyUSB" );
        paternAdd ( "ttyACM" );
        paternAdd ( "ttyGPS" );
        paternAdd ( "refcom" );
    }

 //  Looking for user privilege openable devices in /dev
 //  Fulup use scandir to improve user experience and support new generation of AIS devices.

      wxString sdev;
      int ind, fcount;
      struct dirent **filelist = {0};

      // scan directory filter is applied automatically by this call
      fcount = scandir("/dev", &filelist, paternFilter, alphasort);

      for(ind = 0; ind < fcount; ind++)  {
       wxString sdev (filelist[ind]->d_name, wxConvUTF8);
       sdev.Prepend (_T("/dev/"));

       preturn->Add (sdev);
       free(filelist[ind]);
      }

      free(filelist);

//        We try to add a few more, arbitrarily, for those systems that have fixed, traditional COM ports

#ifdef __linux__
    if( isTTYreal("/dev/ttyS0") )
        preturn->Add( _T("/dev/ttyS0") );

    if( isTTYreal("/dev/ttyS1") )
        preturn->Add( _T("/dev/ttyS1") );
#endif /* linux */


#endif // defined(__UNIX__) && !defined(__WXOSX__)

#ifdef PROBE_PORTS__WITH_HELPER

    /*
     *     For modern Linux/(Posix??) systems, we may use
     *     the system files /proc/tty/driver/serial
     *     and /proc/tty/driver/usbserial to identify
     *     available serial ports.
     *     A complicating factor is that most (all??) linux
     *     systems require root privileges to access these files.
     *     We will use a helper program method here, despite implied vulnerability.
     */

    char buf[256]; // enough to hold one line from serial devices list
    char left_digit;
    char right_digit;
    int port_num;
    FILE *f;

    pid_t pID = vfork();

    if (pID == 0)// child
    {
//    Temporarily gain root privileges
        seteuid(file_user_id);

//  Execute the helper program
        execlp("ocpnhelper", "ocpnhelper", "-SB", NULL);

//  Return to user privileges
        seteuid(user_user_id);

        wxLogMessage(_T("Warning: ocpnhelper failed...."));
        _exit(0);// If exec fails then exit forked process.
    }

    wait(NULL);                  // for the child to quit

//    Read and parse the files

    /*
     * see if we have any traditional ttySx ports available
     */
    f = fopen("/var/tmp/serial", "r");

    if (f != NULL)
    {
        wxLogMessage(_T("Parsing copy of /proc/tty/driver/serial..."));

        /* read in each line of the file */
        while(fgets(buf, sizeof(buf), f) != NULL)
        {
            wxString sm(buf, wxConvUTF8);
            sm.Prepend(_T("   "));
            sm.Replace(_T("\n"), _T(" "));
            wxLogMessage(sm);

            /* if the line doesn't start with a number get the next line */
            if (buf[0] < '0' || buf[0] > '9')
            continue;

            /*
             * convert digits to an int
             */
            left_digit = buf[0];
            right_digit = buf[1];
            if (right_digit < '0' || right_digit > '9')
            port_num = left_digit - '0';
            else
            port_num = (left_digit - '0') * 10 + right_digit - '0';

            /* skip if "unknown" in the string */
            if (strstr(buf, "unknown") != NULL)
            continue;

            /* upper limit of 15 */
            if (port_num > 15)
            continue;

            /* create string from port_num  */

            wxString s;
            s.Printf(_T("/dev/ttyS%d"), port_num);

            /*  add to the output array  */
            preturn->Add(wxString(s));

        }

        fclose(f);
    }

    /*
     * Same for USB ports
     */
    f = fopen("/var/tmp/usbserial", "r");

    if (f != NULL)
    {
        wxLogMessage(_T("Parsing copy of /proc/tty/driver/usbserial..."));

        /* read in each line of the file */
        while(fgets(buf, sizeof(buf), f) != NULL)
        {

            wxString sm(buf, wxConvUTF8);
            sm.Prepend(_T("   "));
            sm.Replace(_T("\n"), _T(" "));
            wxLogMessage(sm);

            /* if the line doesn't start with a number get the next line */
            if (buf[0] < '0' || buf[0] > '9')
            continue;

            /*
             * convert digits to an int
             */
            left_digit = buf[0];
            right_digit = buf[1];
            if (right_digit < '0' || right_digit > '9')
            port_num = left_digit - '0';
            else
            port_num = (left_digit - '0') * 10 + right_digit - '0';

            /* skip if "unknown" in the string */
            if (strstr(buf, "unknown") != NULL)
            continue;

            /* upper limit of 15 */
            if (port_num > 15)
            continue;

            /* create string from port_num  */

            wxString s;
            s.Printf(_T("/dev/ttyUSB%d"), port_num);

            /*  add to the output array  */
            preturn->Add(wxString(s));

        }

        fclose(f);
    }

    //    As a fallback, in case seteuid doesn't work....
    //    provide some defaults
    //    This is currently the case for GTK+, which
    //    refuses to run suid.  sigh...

    if(preturn->IsEmpty())
    {
        preturn->Add( _T("/dev/ttyS0"));
        preturn->Add( _T("/dev/ttyS1"));
        preturn->Add( _T("/dev/ttyUSB0"));
        preturn->Add( _T("/dev/ttyUSB1"));
        preturn->Add( _T("/dev/ttyACM0"));
        preturn->Add( _T("/dev/ttyACM1"));
    }

//    Clean up the temporary files created by helper.
    pid_t cpID = vfork();

    if (cpID == 0)// child
    {
//    Temporarily gain root privileges
        seteuid(file_user_id);

//  Execute the helper program
        execlp("ocpnhelper", "ocpnhelper", "-U", NULL);

//  Return to user privileges
        seteuid(user_user_id);
        _exit(0);// If exec fails then exit forked process.
    }

#endif      //  PROBE_PORTS__WITH_HELPER
    return preturn;
}

#endif   //  defined(OCPN_USE_SYSFS_PORTS) && defined(HAVE_SYSFS_PORTS)

#endif


bool CheckSerialAccess( void )
{
    bool bret = true;
#if defined(__UNIX__) && !defined(__OCPN__ANDROID__)

#if 0
    termios ttyset_old;
    termios ttyset;
    termios ttyset_check;

    // Get a list of the ports
    wxArrayString *ports = EnumerateSerialPorts();
    if( ports->GetCount() == 0 )
        bret = false;

    for(unsigned int i=0 ; i < ports->GetCount() ; i++){
        wxCharBuffer buf = ports->Item(i).ToUTF8();

        //      For the first real port found, try to open it, write some config, and
        //      be sure it reads back correctly.
        if( isTTYreal( buf.data() ) ){
            int fd = open(buf.data(), O_RDWR | O_NONBLOCK | O_NOCTTY);

            // device name is pointing to a real device
            if(fd > 0) {

                if (isatty(fd) != 0)
                {
                    /* Save original terminal parameters */
                    tcgetattr(fd,&ttyset_old);
                    // Write some data
                    memcpy(&ttyset, &ttyset_old, sizeof(termios));

                    ttyset.c_cflag &=~ CSIZE;
                    ttyset.c_cflag |= CSIZE & CS7;

                    tcsetattr(fd, TCSANOW, &ttyset);

                    // Read it back
                    tcgetattr(fd, &ttyset_check);
                    if(( ttyset_check.c_cflag & CSIZE) != CS7 ){
                        bret = false;
                    }
                    else {
                            // and again
                        ttyset.c_cflag &=~ CSIZE;
                        ttyset.c_cflag |= CSIZE & CS8;

                        tcsetattr(fd, TCSANOW, &ttyset);

                            // Read it back
                        tcgetattr(fd, &ttyset_check);
                        if(( ttyset_check.c_cflag & CSIZE) != CS8 ){
                            bret = false;
                        }
                    }

                    tcsetattr(fd, TCSANOW, &ttyset_old);
                }

                close (fd);
            }   // if open
        }
    }

#endif  // 0

    //  Who owns /dev/ttyS0?
    bret = false;

    wxArrayString result1;
    wxExecute(_T("stat -c %G /dev/ttyS0"), result1);
    if(!result1.size())
        wxExecute(_T("stat -c %G /dev/ttyUSB0"), result1);

    if(!result1.size())
        wxExecute(_T("stat -c %G /dev/ttyACM0"), result1);

    wxString msg1 = _("OpenCPN requires access to serial ports to use serial NMEA data.\n");
    if(!result1.size()) {
        wxString msg = msg1 + _("No Serial Ports can be found on this system.\n\
You must install a serial port (modprobe correct kernel module) or plug in a usb serial device.\n");

        OCPNMessageBox ( NULL, msg, wxString( _("OpenCPN Info") ), wxICON_INFORMATION | wxOK, 30 );
        return false;
    }

    //  Is the current user in this group?
    wxString user = wxGetUserId(), group = result1[0];

    wxArrayString result2;
    wxExecute(_T("groups ") + user, result2);

    if(result2.size()) {
        wxString user_groups = result2[0];

        if(user_groups.Find(group) != wxNOT_FOUND)
            bret = true;
    }

#ifdef FLATPAK
    return bret;
#endif

    if(!bret){

        wxString msg = msg1 + _("\
You do currently not have permission to access the serial ports on this system.\n\n\
It is suggested that you exit OpenCPN now,\n\
and add yourself to the correct group to enable serial port access.\n\n\
You may do so by executing the following command from the linux command line:\n\n\
                sudo usermod -a -G ");

        msg += group;
        msg += _T(" ");
        msg += user;
        msg += _T("\n");

        OCPNMessageBox ( NULL, msg, wxString( _("OpenCPN Info") ), wxICON_INFORMATION | wxOK, 30 );
    }

#endif  // (__UNIX__) && !defined(__OCPN__ANDROID__)

    return bret;
}
