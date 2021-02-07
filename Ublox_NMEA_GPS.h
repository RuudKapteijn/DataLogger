class Ublox_NMEA_GPS {
  private:
    String _sentence = "";
    String _dat = "";
    String _tim = "";
    String _lon = "";
    String _lat = "";
    String _cog = "";
    String _sog = "";
    String _alt = "";
    String _fix = "";
    String _siv = "";
     
  public:
    Ublox_NMEA_GPS();
    void    init();
    void    update();
    String  getDAT();
    String  getTIM();
    String  getLON();
    String  getLAT();
    String  getCOG();
    String  getSOG();
    String  getALT();
    String  getFIX();
    String  getSIV();
};

extern long count_RMC;
extern long count_GGA;
