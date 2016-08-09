"""Library for parsing AIS messages.

For more information see http://catb.org/gpsd/AIVDM.html.

See Also:
    http://catb.org/gpsd/AIVDM.html
    http://www.nmea.org/Assets/nmea%20collision%20avoidance%20through%20ais.pdf
    http://www.it-digin.com/blog/?p=20
    
_ais is from https://github.com/schwehr/noaadata/blob/master/contrib/ais.py
    Comments and minor Python 3 changes added.
"""
from pynmea2 import *
from .ais_parser import *
from ._ais import *

parse = NMEASentence.parse
