"""Library for parsing AIS messages.

For more information see http://catb.org/gpsd/AIVDM.html.

See Also:
    http://catb.org/gpsd/AIVDM.html
    https://github.com/schwehr/noaadata/blob/master/contrib/ais.py
    http://www.nmea.org/Assets/nmea%20collision%20avoidance%20through%20ais.pdf

Example:
import pynmea2.ais as ais
v = ais.parse("!AIVDM,1,1,,B,177KQJ5000G?tO`K>RA1wUbN0TKH,0*5C")

v = ais.parse("!AIVDM,2,1,3,B,55P5TL01VIaAL@7WKO@mBplU@<PDhh000000001S;AJ::4A80?4i@E53,0*3E")
v = ais.parse("!AIVDM,2,2,3,B,1@0000000000000,2*55")
"""
import re
import collections

from pynmea2 import NMEASentence, TalkerSentence
from ._ais import BitVector, aivdm_decode, aivdm_unpack


class IncompleteMessageError(ValueError):
    '''
        Inherits from ValueError for distinct Incomplete Sequential Message Exception
    '''
    

# Change the NMEA setntence parser to include [\!] as the start flag.
NMEASentence.sentence_re = re.compile(r'''
        # start of string, optional whitespace, optional '$'
        ^\s*[\$|\!]?

        # message (from '$' or start to checksum or end, non-inclusve)
        (?P<nmea_str>
            # sentence type identifier
            (?P<sentence_type>

                # proprietary sentence
                (P\w{3})|

                # query sentence, ie: 'CCGPQ,GGA'
                # NOTE: this should have no data
                (\w{2}\w{2}Q,\w{3})|

                # taker sentence, ie: 'GPGGA', 'AIVDM', 'AIVDO'
                (\w{2}\w{3},)
            )

            # rest of message
            (?P<data>[^*]*)

        )
        # checksum: *HH
        (?:[*](?P<checksum>[A-F0-9]{2}))?

        # optional trailing whitespace
        \s*[\r\n]*$
        ''', re.X | re.IGNORECASE)


class VDM(TalkerSentence):
    """
    !AI - Mobile AIS station
    
    Example: http://catb.org/gpsd/AIVDM.html
        !AIVDM,1,1,,B,177KQJ5000G?tO`K>RA1wUbN0TKH,0*5C
    
        Field 1, !AIVDM, identifies this as an AIVDM packet.

        Field 2 (1 in this example) is the count of fragments in the currently accumulating message.
        The payload size of each sentence is limited by NMEA 0183's 82-character maximum, so it is 
        sometimes required to split a payload over several fragment sentences.
        
        Field 3 (1 in this example) is the fragment number of this sentence. It will be one-based. 
        A sentence with a fragment count of 1 and a fragment number of 1 is complete in itself.
        
        Field 4 (empty in this example) is a sequential message ID for multi-sentence messages.
        
        Field 5 (B in this example) is a radio channel code. AIS uses the high side of the duplex 
        from two VHF radio channels: AIS Channel A is 161.975Mhz (87B); AIS Channel B is 
        162.025Mhz (88B). In the wild, channel codes 1 and 2 may also be encountered; the standards 
        do not prescribe an interpretation of these but it's obvious enough..
        
        Field 6 (177KQJ5000G?tO`K>RA1wUbN0TKH in this example) is the data payload. We'll describe 
        how to decode this in later sections.
        
        Field 7 (0) is the number of fill bits requires to pad the data payload to a 6 bit 
        boundary, ranging from 0 to 5. Equivalently, subtracting 5 from this tells how many least 
        significant bits of the last 6-bit nibble in the data payload should be ignored. Note that 
        this pad byte has a tricky interaction with the <[ITU-1371]> requirement for byte alignment 
        in over-the-air AIS messages; see the detailed discussion of message lengths and alignment 
        in a later section.
        
        The *-separated suffix (*5C) is the NMEA 0183 data-integrity checksum for the sentence, 
        preceded by "*". It is computed on the entire sentence including the AIVDM tag but 
        excluding the leading "!".
        
        For comparison, here is an example of a multifragment sentence with a nonempty message ID 
        field:
        
        .. code-Block::
        
            !AIVDM,2,1,3,B,55P5TL01VIaAL@7WKO@mBplU@<PDhh000000001S;AJ::4A80?4i@E53,0*3E
            !AIVDM,2,2,3,B,1@0000000000000,2*55
    """

    fields = (
        ("Count of Fragments", "count", int),
        ("Fragment Number", "fragment", int),
        ("Sequential Message ID", "sequential_msg_id"),
        ("Radio Channel Code", "radio_code"),
        ("Payload", "payload"),
        ("Number of Fill Bits", "fill_bits"),
    )

    # Store incomplete sequential messages
    SEQUENTIAL_MSG = collections.OrderedDict()
    SEQ_LIMIT = float("inf")

    # Notify when a sequential message is destroyed before being completed
    NOTIFY = print


    def __init__(self, *args, **kwargs):
        self.sub_fields = tuple()
        super(VDM, self).__init__(*args, **kwargs)

        # Parse the payload
        if self.payload != "":
            self.parse_payload()
    # end Constructor

    def is_complete(self):
        """Return if the message is complete."""
        try:
            return self.count == self.fragment
        except (ValueError, TypeError):
            return False
    # end is_complete

    def parse_payload(self):
        """Parse the payload. Check the count, fragment, and sequential message id and stitch
        together the payload. If this payload completes a message the sub_fields will be populated.

        Warning:
            The message count and fragment must be parsed before this method is called!
            
        Warning:
            May fail and remove partial sequential messages if the number of sequential messsages
            exceeds the limit or a message has the same sequential message id with the fragment 1.
            When this happens NOTIFY is called to alert the user.

        Raises:
            IncompleteMessageError: When a fragment number is retrieved out of turn.
        """
        # Variables
        cls = self.__class__
        self.count = int(self.count)
        self.fragment = int(self.fragment)

        # Handle the sequential message id
        prev_seq = cls.SEQUENTIAL_MSG.get(self.sequential_msg_id, None)
        cls.SEQUENTIAL_MSG[self.sequential_msg_id] = self

        # Remove the oldest if greater than the limit
        if len(cls.SEQUENTIAL_MSG) > cls.SEQ_LIMIT:
            old = list(cls.SEQUENTIAL_MSG)[0]
            cls.SEQUENTIAL_MSG.pop(old)
            cls.NOTIFY("Failed to complete a sequential message with id " + repr(old) +
                       " (limit reached).")

        # Check fragment    
        if self.fragment == 1:
            prev_seq = None

        # Check the previous sequential message
        last = 0
        prev_payload = ""
        if prev_seq is not None:
            last = prev_seq.fragment
            prev_payload = prev_seq.payload

        # Check if this is the expected fragment
        if last + 1 != self.fragment:
            cls.SEQUENTIAL_MSG.pop(self.sequential_msg_id)
            raise IncompleteMessageError("Missing sequential message fragment "+
                                         repr(last + 1)+"!")

        # Add the previous payload
        self.payload = prev_payload + self.payload

        # Check for a complete payload
        if self.is_complete():
            # REMOVE FROM SEQUENTIAL_MSG
            cls.SEQUENTIAL_MSG.pop(self.sequential_msg_id)
            
            # Parse the completed payload
            self.sub_fields = self.parse_complete_payload(self.payload)
            return self
    # end parse_payload

    @staticmethod
    def parse_complete_payload(payload):
        """Parse a complete payload message.

        Args:
            payload (str): A complete payload with all of the required fragments stitched together.
        """
        # Render assembled payload to packed bytes
        bits = BitVector()
        bits.from_sixbit(payload)

        # Magic recursive unpacking operation
        sub_fields = aivdm_unpack(bits, 0, {}, aivdm_decode)

        # Format the data
        # (_ais.py leaves a function or list as the last item. Use it to interpret the data)
        interpreted = [] # varname, value, type, display name, raw value, callback
        for field in sub_fields:
            if len(field) == 5:
                try:
                    val = field[4](field[1])
                except:
                    try:
                        val = field[4][field[1]]
                    except:
                        val = field[1]
                        
                if isinstance(val, str):
                    try:
                        val = int(val)
                    except:
                        try:
                            val = float(val)
                        except:
                            pass
                
                field = [field[0], val, field[2], field[3], field[1],] + field[4:] 
            interpreted.append(list(field))
        # end

        return interpreted
    # end parse_complete_payload
# end VDM

