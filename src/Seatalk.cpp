
#include "wx/wxprec.h"

#ifndef  WX_PRECOMP
  #include "wx/wx.h"
#endif //precompiled headers


#include "dychart.h"
#include "nmea0183/nmea0183.h"
#include "datastream.h"
#include "OCP_DataStreamInput_Thread.h"
#include "OCPN_DataStreamEvent.h"
#include "Seatalk.h"


#define DS_RX_BUFFER_SIZE 4096


#ifdef __POSIX__
#include <sys/termios.h>
#endif

OCP_StkDataStreamInput_Thread::OCP_StkDataStreamInput_Thread(DataStream *Launcher,
								wxEvtHandler *MessageTarget,
								const wxString& PortName,
								const wxString& strBaudRate,
								wxMutex *pout_mutex,
								dsPortType io_select):
	OCP_DataStreamInput_Thread( Launcher,
							MessageTarget,
							PortName,
							strBaudRate,
							pout_mutex,
							io_select){}
							
OCP_StkDataStreamInput_Thread::~OCP_StkDataStreamInput_Thread(void)	{}
							
void *OCP_StkDataStreamInput_Thread::Entry()
{

    bool not_done = true;
    bool nl_found;
    wxString msg;
	StkToNmea* s2n = new StkToNmea() ;
	wxString tempo;
	b=false;
    //    Request the com port from the comm manager
    if ((m_gps_fd = OpenComPortPhysical(m_PortName, m_baud)) < 0)
    {
       wxString msg(_T("NMEA input device open failed: "));
        msg.Append(m_PortName);
        ThreadMessage(msg);
        goto thread_exit;
    }

    m_launcher->SetSecThreadActive();               // I am alive

//    The main loop

    while((not_done) && (m_launcher->m_Thread_run_flag > 0))
    {
        if(TestDestroy())
            not_done = false;                               // smooth exit

      //    Blocking, timeout protected read of one character at a time
      //    Timeout value is set by c_cc[VTIME]
      //    Storing incoming characters in circular buffer
      //    And watching for new line character
      //     On new line character, send notification to parent
        char next_byte = 0;
        ssize_t newdata;
        newdata = read(m_gps_fd, &next_byte, 1);            // read of one char
                                                            // return (-1) if no data available, timeout

#ifdef __WXOSX__
        if (newdata < 0 )
            wxThread::Sleep(100) ;
#endif


      // Fulup patch for handling hot-plug or wakeup events
      // from serial port drivers
        {
              static int maxErrorLoop;

              if (newdata > 0)
              {
        // we have data, so clear error
                    maxErrorLoop =0;
              }
              else
              {
        // no need to retry every 1ms when on error
                    sleep (1);

        // if we have more no character for 5 second then try to reopen the port
                    if (maxErrorLoop++ > 5)
                    {

        // do not retry for the next 5s
                          maxErrorLoop = 0;

        // free old unplug current port
                          CloseComPortPhysical(m_gps_fd);
        //    Request the com port from the comm manager
                          if ((m_gps_fd = OpenComPortPhysical(m_PortName, m_baud)) < 0)  {
//                                wxString msg(_T("NMEA input device open failed (will retry): "));
//                                msg.Append(m_PortName);
//                                ThreadMessage(msg);
                          } else {
                                wxString msg(_T("NMEA input device open on hotplug OK: "));
                                msg.Append(m_PortName);
                                ThreadMessage(msg);
                          }
                    }
              }
        }

        //  And process any character

        if(newdata > 0)
        {
            nl_found = false;
            *put_ptr++ = next_byte;
            if((put_ptr - rx_buffer) > DS_RX_BUFFER_SIZE)
            put_ptr = rx_buffer;
// this is working only with POSIX Linux serial driver
				bool complete ;
				switch ((unsigned char)next_byte)
				{
					case 0xff:
					if (ind==0)
					{
						ind= 1;
						break;
						}
					
					case 0x00:
					if (ind==1)
					{
						ind= 2;
						break;
						}
					
					
					default:
					if (ind ==2 ) /*  erreur parity impaire */
					{
						if ( getParity(next_byte) )
						{ // even cde
						complete = seatalk(next_byte,1);
						}else 
						{ //odd data
						complete = seatalk(next_byte,0);
						}
					}else
					{
						if ( getParity(next_byte) )
						{ // even data
						complete = seatalk(next_byte,0);
						}else 
						{ //odd cde
						complete = seatalk(next_byte,1);
						}
					}						
					ind =0 ;
					if (complete)
					{
					//wxLogMessage(recu);
					
					tempo = s2n->Decode(buftmp);
					Parse_And_Send_Posn(tempo.mb_str());
					}
					if(s2n->AWupdated) 
						{
							tempo= s2n->TrueWind();
							Parse_And_Send_Posn(tempo.mb_str());
							s2n->AWupdated=false;
						}
					break;
				}
		}
        //      Check for any pending output message

        if( m_pout_mutex && (wxMUTEX_NO_ERROR == m_pout_mutex->TryLock()) ){
            bool b_qdata = (m_takIndex != (-1) || m_putIndex != (-1));
            
            while(b_qdata){
                if(m_takIndex < OUT_QUEUE_LENGTH) {
                    
                    //  Take a copy of message
                    char msg[MAX_OUT_QUEUE_MESSAGE_LENGTH];
                    strncpy( msg, m_poutQueue[m_takIndex], MAX_OUT_QUEUE_MESSAGE_LENGTH-1 );
                    
                    //  Update and release the taker index
                    if(m_takIndex==m_putIndex)
                        m_takIndex=m_putIndex=(-1);
                    else if(m_takIndex == (OUT_QUEUE_LENGTH-1) )
                        m_takIndex=0;
                    else
                        m_takIndex++;
                    
                    
                    m_pout_mutex->Unlock();
                    WriteComPortPhysical(m_gps_fd, msg);
                    
                    if( wxMUTEX_NO_ERROR == m_pout_mutex->TryLock() )
                        b_qdata = (m_takIndex != (-1) || m_putIndex != (-1));
                    else
                        b_qdata = false;
                }
                else {                                  // some index error
                    m_takIndex = (-1);
                    m_putIndex = (-1);
                    b_qdata = false;
                }
                
                
            } //while b_qdata
            m_pout_mutex->Unlock();
        }
        
 bail_output:
    bool bail = true;
    }                          // the big while...

//          Close the port cleanly
    CloseComPortPhysical(m_gps_fd);

thread_exit:
    m_launcher->SetSecThreadInActive();             // I am dead
    m_launcher->m_Thread_run_flag = -1;

    return 0;
}

#ifdef __POSIX__

int OCP_StkDataStreamInput_Thread::OpenComPortPhysical(const wxString &com_name, int baud_rate)
{

    // Declare the termios data structures
    termios ttyset_old;
    termios ttyset;

    // Open the serial port.
    int com_fd;
    if ((com_fd = open(com_name.mb_str(), O_RDWR|O_NONBLOCK|O_NOCTTY)) < 0)
        return com_fd;

    speed_t baud_parm;
    switch(baud_rate)
    {
        case 4800:
              baud_parm = B4800;
              break;
        case 9600:
              baud_parm = B9600;
              break;
        case 38400:
              baud_parm = B38400;
              break;
        default:
              baud_parm = B4800;
              break;
    }

    if (isatty(com_fd) != 0)
    {
        /* Save original terminal parameters */
        if (tcgetattr(com_fd,&ttyset_old) != 0)
            return -128;

        memcpy(&ttyset, &ttyset_old, sizeof(termios));

    //  Build the new parms off the old

    //  Baud Rate
        cfsetispeed(&ttyset, baud_parm);
        cfsetospeed(&ttyset, baud_parm);

        tcsetattr(com_fd, TCSANOW, &ttyset);

    // Set blocking/timeout behaviour
        memset(ttyset.c_cc,0,sizeof(ttyset.c_cc));
        ttyset.c_cc[VTIME] = 5;                        // 0.5 sec timeout
        fcntl(com_fd, F_SETFL, fcntl(com_fd, F_GETFL) & !O_NONBLOCK);

    // No Flow Control

        ttyset.c_cflag &= ~(PARENB | PARODD | CRTSCTS);
        ttyset.c_cflag |= CREAD | CLOCAL;
        ttyset.c_iflag = ttyset.c_oflag = ttyset.c_lflag = (tcflag_t) 0;

        int stopbits = 1;
        char parity = 'E';
        ttyset.c_iflag &=~ (PARMRK | INPCK);
        ttyset.c_cflag &=~ (CSIZE | CSTOPB | PARENB | PARODD);
        ttyset.c_cflag |= (stopbits==2 ? CS7|CSTOPB : CS8);
        switch (parity)
        {
            case 'E':
                ttyset.c_iflag |= (PARMRK | INPCK);
                ttyset.c_cflag |= PARENB;
                break;
            case 'O':
                ttyset.c_iflag |= INPCK;
                ttyset.c_cflag |= PARENB | PARODD;
                break;
        }
        ttyset.c_cflag &=~ CSIZE;
        ttyset.c_cflag |= (CSIZE & (stopbits==2 ? CS7 : CS8));
        if (tcsetattr(com_fd, TCSANOW, &ttyset) != 0)
            return -129;

        tcflush(com_fd, TCIOFLUSH);
    }

    return com_fd;
}




bool OCP_StkDataStreamInput_Thread::seatalk(unsigned char d, bool cde) 
{
	int i;
	status= false;
	if (cde)
	{ 
		cpt = 255;
		buftmp[0]= d;
		//wxLogMessage(wxT("cde:  " )+ wxString::Format(_T("%2x"),d));
		b= true ;
		
	}else
	{
	if (b){
		if (cpt == 254)	{
			len = (d & 0x0f)+2;
			cpt = len;
			}
		buftmp[(len -cpt)+1]= d ;
		}	
	}
	if ( !--cpt and b ){
		recu.Clear();
		recu += wxT("** Con_ga ** " );
		for (i=0;i<=len;i++)
			{
				recu += wxString::Format(_T("%2x"),buftmp[i]);
				recu += wxString::FromAscii( ' ');
			}
		
		status= true;
		b=false; //debug
	} 
	return status; // commande complete
	
}

bool OCP_StkDataStreamInput_Thread::getParity(unsigned int n)
{
    bool parity = 0;
    while (n)
    {
        parity = !parity;
        n      = n & (n - 1);
    }
    return !parity;
}

#endif     // posix

//*********************************************
//
//
//

StkToNmea::StkToNmea()
{
	};
	
StkToNmea::~StkToNmea()
 {        
	 };


wxString StkToNmea::Decode(unsigned char tre[255])
{
	NMEA0183 cm_nmea ;
	int r;
	float t;
	SENTENCE snt ;
	wxString unit;
	wxString tk;
	
	tk = wxT("EC");
	cm_nmea.TalkerID= tk ;
	
switch (tre[0])
{
	
		case 0x27 :  //Water temp
		t =  (float)((tre[3]*256 + tre[2])-100)/10 ;
			cm_nmea.Mtw.Empty();
			cm_nmea.Mtw.Temperature= t;
			unit = wxT("C");
			cm_nmea.Mtw.UnitOfMeasurement = unit;
			cm_nmea.Mtw.Write(snt);
			break;
			
		case 0x10 : // ApW direction
			VentAngle =  ((unsigned long)(tre[2]*256 +(unsigned long) tre[3]))/2 ;
		break;
		case 0x11 : // ApW direction
		//(XX & 0x7F) + Y/10 Knots
			VentVitesse =  ((float)(tre[2]& 0x7F) +( (float) tre[3]/10)) ;
			cm_nmea.Mwv.Empty();
			cm_nmea.Mwv.WindAngle= VentAngle;
			unit = wxT("R");
			cm_nmea.Mwv.Reference = unit;
			cm_nmea.Mwv.WindSpeed= VentVitesse;
			if ((tre[2]& 0x80) ==0)cm_nmea.Mwv.WindSpeedUnits= wxT("K");
			cm_nmea.Mwv.IsDataValid = NTrue;
			cm_nmea.Mwv.Write(snt);
			AWupdated=true;
			break;
		
		case 0x00 : // depth
			t =  ((float)tre[4]*256 +(float) tre[3])/10 ;
			cm_nmea.Dpt.Empty();
			cm_nmea.Dpt.DepthMeters = t * 0.3 ;
			cm_nmea.Dpt.OffsetFromTransducerMeters = 0;
			cm_nmea.Dpt.Write(snt);
			break;
		
		case 0x84 :
		HeadingMag =   ((tre[1]>>4) & 0x3)* 90 + (tre[2] & 0x3F)* 2 ;
		
		break;
		
		case 0x53 :
		Cog =   ((tre[1]>>4) & 0x3)* 90 + (tre[2] & 0x3F)* 2 + (float)((tre[1])>>4 & 0xc)/ 2;
		break;
		case 0x20 :
		Sow =  ((float)tre[3]*256 +(float) tre[2])/10 ;
		cm_nmea.Vhw.DegreesTrue       = Cog;
        cm_nmea.Vhw.DegreesMagnetic   = HeadingMag;
        cm_nmea.Vhw.Knots             = Sow;
        cm_nmea.Vhw.KilometersPerHour = Sow *1.852;
		cm_nmea.Vhw.Write(snt);
		break;
		
		
		case 0x99 : // hdg
		r= (int)(signed char)tre[2];
		cm_nmea.Hdg.Empty();
		if (r>0)
		{
				cm_nmea.Hdg.MagneticVariationDegrees = abs(r)  ;
				cm_nmea.Hdg.MagneticVariationDirection = West  ;
			}
		else
			{
				cm_nmea.Hdg.MagneticVariationDegrees = abs(r)  ;
				cm_nmea.Hdg.MagneticVariationDirection = East  ;
			}
		break;
		
		case 0x9c : // hdg
		HeadingMag =   ((tre[1]>>4) & 0x3)* 90 + (tre[2] & 0x3F)* 2 ;
		cm_nmea.Hdg.MagneticSensorHeadingDegrees = HeadingMag  ;
		cm_nmea.Hdg.MagneticDeviationDegrees  = 0  ;
		cm_nmea.Hdg.MagneticDeviationDirection = East  ;
		cm_nmea.Hdg.Write(snt);
		
		break;
		
		default:
		break;
			
		
} // end switch
return snt.Sentence ;
}

wxString StkToNmea::TrueWind()
{
	NMEA0183 cm_nmea ;
	int r;
	SENTENCE snt ;
	wxString unit;
	wxString tk;
	
			tk = wxT("EC");
			cm_nmea.TalkerID= tk ;
			// true wind calculation
			cm_nmea.Mwv.Empty();
			cm_nmea.Mwv.WindAngle= VentAngle;
			unit = wxT("T");
			cm_nmea.Mwv.Reference = unit;
			cm_nmea.Mwv.WindSpeed= VentVitesse;
			cm_nmea.Mwv.WindSpeedUnits= wxT("K");
			cm_nmea.Mwv.IsDataValid = NTrue;
			cm_nmea.Mwv.Write(snt);
			return snt.Sentence ;
}