
#include "wx/wxprec.h"

#ifndef  WX_PRECOMP
  #include "wx/wx.h"
#endif //precompiled headers

#include "Seatalk.h"

#include "nmea0183/nmea0183.h"


StkToNmea::StkToNmea()
{
	};
	
StkToNmea::~StkToNmea()
 {        
	 };


wxString StkToNmea::Decode(unsigned char tre[255])
{
	NMEA0183 cm_nmea ;
	float t;
	int r;
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
			vent =  ((unsigned long)(tre[2]*256 +(unsigned long) tre[3]))/2 ;
		break;
		case 0x11 : // ApW direction
		//(XX & 0x7F) + Y/10 Knots
			t =  ((float)(tre[2]& 0x7F) +( (float) tre[3]/10)) ;
			cm_nmea.Mwv.Empty();
			cm_nmea.Mwv.WindAngle= vent;
			unit = wxT("R");
			cm_nmea.Mwv.Reference = unit;
			cm_nmea.Mwv.WindSpeed= t;
			if ((tre[2]& 0x80) ==0)cm_nmea.Mwv.WindSpeedUnits= wxT("K");
			cm_nmea.Mwv.IsDataValid = NTrue;
			cm_nmea.Mwv.Write(snt);
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
