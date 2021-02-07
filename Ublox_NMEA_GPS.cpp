#include <Arduino.h>
#include "Ublox_NMEA_GPS.h"

Ublox_NMEA_GPS::Ublox_NMEA_GPS() {
  _sentence = "";
}

void Ublox_NMEA_GPS::init() {
  Serial1.begin(9600);
}

void Ublox_NMEA_GPS::update() {
  int i=0, wc=0;

  if (_sentence.length() > 100 || !_sentence.startsWith("$")) _sentence = "";   // remove garbage
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (c == '\n') {
      if (_sentence.startsWith("$GNRMC") && _sentence.length() < 100) {
        count_RMC++;
        // Serial.println("INFO: "+_sentence);

        for (i=0; i < _sentence.length() && wc < 1; i++)            // skip to first word after $GP...
          if (_sentence[i] == ',') wc++;

        _tim = "";
        while (i < _sentence.length() && _sentence[i] != ',')       // decode time
          _tim += _sentence[i++];
        wc++;
        i++;

        while (i < _sentence.length() && wc < 3)                    // skip to third word after $GP...
          if (_sentence[i++] == ',') wc++;
        
        _lat = "";
        while (i < _sentence.length() && _sentence[i] != ',')       // decode _lat
          _lat += _sentence[i++];
        wc++;
        i++;
        while (i < _sentence.length() && _sentence[i] != ',')
          _lat += _sentence[i++];
        wc++;
        i++;

        _lon = "";
        while (i < _sentence.length() && _sentence[i] != ',')       // decode _lon
          _lon += _sentence[i++];
        wc++;
        i++;
        while (i < _sentence.length() && _sentence[i] != ',')
          _lon += _sentence[i++];
        wc++;
        i++;

        _sog = "";
        while (i < _sentence.length() && _sentence[i] != ',')       // decode _sog
          _sog += _sentence[i++];
        wc++;
        i++;

        _cog = "";
        while (i < _sentence.length() && _sentence[i] != ',')       // decode _cog
          _cog += _sentence[i++];
        wc++;
        i++;

        _dat = "";
        while (i < _sentence.length() && _sentence[i] != ',')       // decode _dat
          _dat += _sentence[i++];
        wc++;
        i++;

        _sentence = "";
      }
      if (_sentence.startsWith("$GNGGA") && _sentence.length() < 100) {
        count_GGA++;
        //Serial.println("INFO: "+_sentence);

        for (i=0; i < _sentence.length() && wc < 6; i++)            // skip to sixth word after $GP...
          if (_sentence[i] == ',') wc++;

        _fix = "";
        while (i < _sentence.length() && _sentence[i] != ',')       // decode _fix
          _fix += _sentence[i++];
        wc++;
        i++;

        _siv = "";
        while (i < _sentence.length() && _sentence[i] != ',')       // decode _siv
          _siv += _sentence[i++];
        wc++;
        i++;

        while (i < _sentence.length() && wc < 9)                    // skip to nineth word after $GP...
          if (_sentence[i++] == ',') wc++;

        _alt = "";
        while (i < _sentence.length() && _sentence[i] != ',')       // decode _alt
          _alt += _sentence[i++];
        wc++;
        i++;

        _sentence = "";
      }
      if (!_sentence.startsWith("$GNGGA") && !_sentence.startsWith("$GNRMC")) {
        // Serial.println("-/-" + _sentence);
        _sentence = "";
      }
    } else {
      _sentence += c;
    }
  }
  if (_sentence.length() > 100) {
    Serial.println("Sentence length breach - " + _sentence);
    _sentence = "";
  }
}

String Ublox_NMEA_GPS::getTIM() {
  return _tim;
}

String Ublox_NMEA_GPS::getLAT() {
  return _lat;
}

String Ublox_NMEA_GPS::getLON() {
  return _lon;
}

String Ublox_NMEA_GPS::getSOG() {
  return _sog;
}

String Ublox_NMEA_GPS::getCOG() {
  return _cog;
}

String Ublox_NMEA_GPS::getDAT() {
  return _dat;
}

String Ublox_NMEA_GPS::getFIX() {
  return _fix;
}

String Ublox_NMEA_GPS::getSIV() {
  return _siv;
}

String Ublox_NMEA_GPS::getALT() {
  return _alt;
}
