#!/usr/bin/env python3
from enum import IntEnum

from amaranth import DomainRenamer, Elaboratable, Module, ResetInserter, Signal
from apollo_fpga.gateware.advertiser import (
    ApolloAdvertiser,
    ApolloAdvertiserRequestHandler,
)
from luna import top_level_cli
from luna.gateware.architecture.car import LunaECP5DomainGenerator
from luna.gateware.architecture.flash_sn import ECP5FlashUIDStringDescriptor
from luna.gateware.interface.ulpi import UTMITranslator
from luna.gateware.stream.generator import StreamSerializer
from luna.gateware.usb.request.control import ControlRequestHandler
from luna.gateware.usb.request.windows import (
    MicrosoftOS10DescriptorCollection,
    MicrosoftOS10RequestHandler,
)
from amaranth.build           import Attrs, Pins, PinsN, Resource, Subsignal
from amaranth.hdl.rec         import Record
from luna.gateware.usb.stream import USBInStreamInterface
from luna.gateware.usb.usb2.device import USBDevice
from luna.usb2 import USBDevice, USBStreamInEndpoint
from luna_soc.gateware.csr import (
    InFIFOInterface,
    OutFIFOInterface,
    SetupFIFOInterface,
    USBDeviceController,
)
from usb_protocol.emitters import DeviceDescriptorCollection
from usb_protocol.emitters.descriptors.standard import get_string_descriptor
from usb_protocol.types import USBRequestRecipient, USBRequestType
from usb_protocol.types.descriptors.microsoft10 import RegistryTypes

from .analyzer import USBAnalyzer
from .fifo import AsyncFIFOReadReset, HyperRAMPacketFIFO, Stream16to8, StreamFIFO
import cynthion

# This code is largely hacked up gsg analyzer gateware. I only need FULL speed. If basic peripherals evolve beyond that, I'd be amazed.
# I might rely on that though, I need to manipulate packets quick, but I need to buffer ~62 bytes. The block ram is 12 bytes. 
# Idk those other memory buffers yet. What do I do in spec to delay? Maybe nothing. Can I even add another buffer? No lag buffer?
# Maybe it will be clone and replay. We'll have access to both sides.
USB_SPEED_FULL       = 0b01

USB_VENDOR_ID        = cynthion.shared.usb.bVendorId.cynthion
USB_PRODUCT_ID       = cynthion.shared.usb.bProductId.cynthion

BULK_ENDPOINT_NUMBER  = 1
BULK_ENDPOINT_ADDRESS = 0x80 | BULK_ENDPOINT_NUMBER
MAX_BULK_PACKET_SIZE  = 512

class USBAnalyzerVendorRequests(IntEnum):
    GET_STATE = 0
    SET_STATE = 1
    GET_SPEEDS = 2
    SET_TEST_CONFIG = 3
    
class USBAnalyzerRegister(Elaboratable):
    def __init__(self, reset=0x00):
        self.current = Signal(8, reset=reset)
        self.next = Signal(8)
        self.write = Signal()

    def elaborate(self, platform):
        m = Module()
        with m.If(self.write):
            m.d.sync += self.current.eq(self.next)
        return m

class DumbProxyApplet(Elaboratable): 
    def __init__(self):
        
        # This is your debug console. TODO: Wire this up to logging.
        self.uart0_pins = Record([
            ('rx', [('i', 1)]),
            ('tx', [('o', 1)])
        ])
        self.uart1_pins = Record([
            ('rx', [('i', 1)]),
            ('tx', [('o', 1)])
        ])
         
    def create_descriptors(self, platform, sharing):
        major, minor = platform.version
        descriptors = DeviceDescriptorCollection()

        with descriptors.DeviceDescriptor() as d:
            d.idVendor           = USB_VENDOR_ID
            d.idProduct          = USB_PRODUCT_ID

            d.iManufacturer      = "Cynthion Project"
            d.iProduct           = "USB Analyzer"
            d.iSerialNumber      = ECP5FlashUIDStringDescriptor
            d.bcdDevice          = major + (minor * 0.01)

            d.bNumConfigurations = 1
            
            
        with descriptors.ConfigurationDescriptor() as c:
            with c.InterfaceDescriptor() as i:
                i.bInterfaceNumber = 0
                i.bInterfaceClass = 0xFF
                i.bInterfaceSubclass = cynthion.shared.usb.bInterfaceSubClass.analyzer
                i.bInterfaceProtocol = cynthion.shared.usb.bInterfaceProtocol.analyzer

                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = BULK_ENDPOINT_ADDRESS
                    e.wMaxPacketSize   = MAX_BULK_PACKET_SIZE

            if sharing is not None:
                with c.InterfaceDescriptor() as i:
                    i.bInterfaceNumber = 1
                    i.bInterfaceClass = 0xFF
                    i.bInterfaceSubclass = cynthion.shared.usb.bInterfaceSubClass.apollo
                    i.bInterfaceProtocol = ApolloAdvertiserRequestHandler.PROTOCOL_VERSION

        return descriptors


    def elaborate(self, platform):
        m = Module()
        # I made a custom ram bus we'll see if it works
        #ram_bus         = platform.request('ram1')
        
        m.submodules.state = state = USBAnalyzerRegister()
        m.submodules.test_config = test_config = USBAnalyzerRegister(reset=0x01)
        clocking = LunaECP5DomainGenerator()
        m.submodules.clocking = clocking
                                                                                              # you are here
        ulpi = platform.request("target_phy")                                                 #      |
        # We pass this to the analyzer later. This thing is the target device (keyboard-mouse-etc->target) and I think you can share it and do weird stuff.
        m.submodules.utmi = utmi = UTMITranslator(ulpi=ulpi)
        
        # I think I read I can do this and maintain control? This powers target A on the device:
        # VBUS on each of the Type-C ports can be connected to TARGET A through
        # a bidirectional switch. If any of these switches is enabled, TARGET A
        # is considered an output. An additional switch can be enabled to pass
        # VBUS through to another port in addition to TARGET A.
        
        # I wonder if this is what the guy meant by hardwire. But if I'm synced (which im figuring out), does it matter? Could I add another device?
        # This is what powers the target in the analyze example. How do I pass "An additional switch can be enabled to pass VBUS through to another port in addition to TARGET A."
        platform.request("target_c_vbus_en").o  .eq(1)
        # We're goto route power to the aux bus? 
        platform.request("aux_vbus_en").o  .eq(1)
        

        m.d.comb += [
            # We actually want to pump the packet, just from another queue
            # Removing this makes it passive, but I wonder if we could still write. Somewhere the docs say you can go for it but it's undefined I think.
            # how big is the buffer? I think I saw 16bits? Right there could I inject x packets later knowing I have those bits and swap inline.
            # but I want to build a buffer in between where I can hold a packet and play with it. From their source you don't have to terminate. If you follow 
            # the clock you could probably inject right there. 
            # utmi.op_mode     .eq(0b01),
            utmi.xcvr_select .eq(state.current[1:3]),
        ]
        
        # This here has nothing to do with the above I don't think. I think this is the apollo mirror, I'm only coding for the cynthion, so we use the control_phy. I still don't quite know where I'm going yet.
        uplink_ulpi = platform.request("control_phy")
        m.submodules.usb = usb = USBDevice(bus=uplink_ulpi)
        sharing = platform.port_sharing("control_phy")
        
        descriptors = self.create_descriptors(platform, sharing)
        
        descriptors.add_descriptor(get_string_descriptor("MSFT100\xee"), index=0xee)
        msft_descriptors = MicrosoftOS10DescriptorCollection()
        with msft_descriptors.ExtendedCompatIDDescriptor() as c:
            with c.Function() as f:
                f.bFirstInterfaceNumber = 0
                f.compatibleID          = 'WINUSB'
            if sharing is not None:
                with c.Function() as f:
                    f.bFirstInterfaceNumber = 1
                    f.compatibleID          = 'WINUSB'
        with msft_descriptors.ExtendedPropertiesDescriptor() as d:
            with d.Property() as p:
                p.dwPropertyDataType = RegistryTypes.REG_SZ
                p.PropertyName       = "DeviceInterfaceGUID"
                p.PropertyData       = "{88bae032-5a81-49f0-bc3d-a4ff138216d6}"

        control_endpoint = usb.add_standard_control_endpoint(descriptors, avoid_blockram=True)

        msft_handler = MicrosoftOS10RequestHandler(msft_descriptors, request_code=0xee)
        control_endpoint.add_request_handler(msft_handler)

        vendor_request_handler = USBAnalyzerVendorRequestHandler(state, test_config)
        control_endpoint.add_request_handler(vendor_request_handler)

        if sharing == "advertising":
            adv = m.submodules.adv = ApolloAdvertiser()
            control_endpoint.add_request_handler(adv.default_request_handler(1))

        # I think this is how they log in packetry, but that means that vendor request can be pretty darn big. 62 bytes. We might be able to store that.
        # https://www.latticesemi.com/Products/FPGAandCPLD/ECP5
        # LFE5U-12 - 
        # LUTs (K)                : 12
        # sysMEM Blocks (18 Kbits): 32
        # Embedded Memory (Kbits) : 576
        # Distributed RAM Bits    : 97
        # LUTs (K)                : 12 K - What the heck is a K? its like an angry text. "k".
        
        # Is this right? it might be 1000? but this is a lot of space. It's probably slow? Can I get to this though?
        # sysMEM Blocks (18 Kbits): 32 * 18 Kbits = 32 * 18 * 1024 / 8 = 73728 bytes?
        # Embedded Memory (Kbits) : 576 Kbits = 576 * 1024 / 8 = 73728 bytes?
        # Distributed RAM Bits    : 97 
        
        stream_ep = USBStreamInEndpoint(
            endpoint_number=BULK_ENDPOINT_NUMBER,
            max_packet_size=MAX_BULK_PACKET_SIZE
        )
        usb.add_endpoint(stream_ep)

        # Analyzer was fast, but was "backed by a small ringbuffer in FPGA block RAM." I think thats 97. 12 bytes?
        # Looks like handling a packet is pretty topped out. But they support super speed. Is system memory fast enough for these clock cycles? idk enough.
        # It's backed by the 4/12 bytes of that ring buffer in that utmi_interface alone. There's something about header and event. Idk if those are allocated or just
        # calculated yet. If they allocate, theres 12/12 bytes, and I have to fall back to inline analyzer and hope I can still inject.
        m.submodules.analyzer = analyzer = USBAnalyzer(utmi_interface=utmi)

        # I think it's just 4. Which is good because I can start another one and glue them together. Part of those is headers, so maybe I can squeeze 4 for a replacement data buffer. Even if only 2.
        reset_on_start = ResetInserter(analyzer.discarding)
        m.submodules.psram_fifo = psram_fifo = reset_on_start(
            HyperRAMPacketFIFO(out_fifo_depth=4096))

        m.submodules.s16to8 = s16to8 = reset_on_start(Stream16to8())

        m.submodules.clk_conv = clk_conv = StreamFIFO(
            AsyncFIFOReadReset(width=8, depth=4, r_domain="usb", w_domain="sync"))

        m.d.comb += [
            # Idk yet what this does. We did not signal passive earlier and give up control.
            # This is like syncing the clock which I don't really get. Like I get a clock, but we didn't give up control so who has it set?
            analyzer.capture_enable     .eq(state.current[0]),

            stream_ep.flush             .eq(analyzer.idle & ~analyzer.capture_enable),

            stream_ep.discard           .eq(analyzer.discarding),

            # USB stream pipeline.
            psram_fifo.input            .stream_eq(analyzer.stream),
            s16to8.input                .stream_eq(psram_fifo.output),
            clk_conv.input              .stream_eq(s16to8.output),
            clk_conv.fifo.ext_rst       .eq(analyzer.discarding),
            stream_ep.stream            .stream_eq(clk_conv.output),

            usb.connect                 .eq(1),

            # LED indicators.
            platform.request("led", 0).o  .eq(analyzer.capturing),
            platform.request("led", 1).o  .eq(stream_ep.stream.valid),

            platform.request("led", 3).o  .eq(utmi.session_valid),
            platform.request("led", 4).o  .eq(utmi.rx_active),
            platform.request("led", 5).o  .eq(utmi.rx_error),
        ]
        
        
        # # Thats a single analyzer mirroring a socket I never let go. I need another one for aux, but this is where I don't really get the clocks. 
        # # I think this is where I will come to understand ep0?. Idk im just going to try if you got it, ship it, if ya don't skip it. Maybe the spec does not allow that.
        #                                     # you are here         
        # ulpi1 = platform.request("aux_phy") #      |
        # # This thing is the aux device (keyboard-mouse-etc->target) 
        # # Someone has to control the clock? Is this ep0? We'll figure it out.
        # # m.submodules.utmi1 = utmi1 = UTMITranslator(ulpi=ulpi1,  handle_clocking=False)
        # m.submodules.utmi1 = utmi1 = UTMITranslator(ulpi=ulpi1)
        # m.submodules.analyzer1 = analyzer1 = USBAnalyzer(utmi_interface=utmi1)

        # reset_on_start1 = ResetInserter(analyzer1.discarding)
        # m.submodules.psram_fifo1 = psram_fifo1 = reset_on_start1(
        #     HyperRAMPacketFIFO(out_fifo_depth=4096, ram_bus=ram_bus))

        # m.submodules.s16to81 = s16to81 = reset_on_start1(Stream16to8())

        # m.submodules.clk_conv1 = clk_conv1 = StreamFIFO(
        #     AsyncFIFOReadReset(width=8, depth=4, r_domain="usb", w_domain="sync"))

        # m.d.comb += [
        #     # Idk yet what this does. We did not signal passive earlier and give up control.
        #     # This is like syncing the clock which I don't really get. Like I get a clock, but we didn't give up control so who has it set?
        #     analyzer1.capture_enable     .eq(state.current[0]),

        #     # USB stream pipeline.
        #     psram_fifo1.input            .stream_eq(analyzer1.stream),
        #     s16to81.input                .stream_eq(psram_fifo1.output),
        #     clk_conv1.input              .stream_eq(s16to81.output),
        #     clk_conv1.fifo.ext_rst       .eq(analyzer1.discarding),
        #     # stream_ep.stream            .stream_eq(clk_conv1.output),

        #     usb.connect                 .eq(1),

        #     # LED indicators.
        #     platform.request("led", 2).o  .eq(analyzer1.capturing),
        # ]
        
        
        return m

class USBAnalyzerVendorRequestHandler(ControlRequestHandler):
    def __init__(self, state, test_config):
        self.state = state
        self.test_config = test_config
        super().__init__()

    def elaborate(self, platform):
        m = Module()
        interface = self.interface

        # Create convenience aliases for our interface components.
        setup               = interface.setup
        handshake_generator = interface.handshakes_out

        # Transmitter for small-constant-response requests
        m.submodules.transmitter = transmitter = \
            StreamSerializer(data_length=1, domain="usb", stream_type=USBInStreamInterface, max_length_width=1)

        # Handle vendor requests to our interface.
        with m.If(
                (setup.type == USBRequestType.VENDOR) &
                (setup.recipient == USBRequestRecipient.INTERFACE) &
                (setup.index == 0)):

            m.d.comb += interface.claim.eq(
                (setup.request == USBAnalyzerVendorRequests.GET_STATE) |
                (setup.request == USBAnalyzerVendorRequests.SET_STATE) |
                (setup.request == USBAnalyzerVendorRequests.GET_SPEEDS)|
                (setup.request == USBAnalyzerVendorRequests.SET_TEST_CONFIG))

            with m.FSM(domain="usb"):

                # IDLE -- not handling any active request
                with m.State('IDLE'):

                    # If we've received a new setup packet, handle it.
                    with m.If(setup.received):

                        # Select which vendor we're going to handle.
                        with m.Switch(setup.request):

                            with m.Case(USBAnalyzerVendorRequests.GET_STATE):
                                m.next = 'GET_STATE'
                            with m.Case(USBAnalyzerVendorRequests.SET_STATE):
                                m.next = 'SET_STATE'
                            with m.Case(USBAnalyzerVendorRequests.GET_SPEEDS):
                                m.next = 'GET_SPEEDS'
                            with m.Case(USBAnalyzerVendorRequests.SET_TEST_CONFIG):
                                m.next = 'SET_TEST_CONFIG'

                # GET_STATE -- Fetch the device's state
                with m.State('GET_STATE'):
                    self.handle_simple_data_request(m, transmitter, self.state.current, length=1)

                # SET_STATE -- The host is trying to set our state
                with m.State('SET_STATE'):
                    self.handle_register_write_request(m, self.state.next, self.state.write)

                # GET_SPEEDS -- Fetch the device's supported USB speeds
                with m.State('GET_SPEEDS'):
                    supported_speeds = USB_SPEED_FULL
                    self.handle_simple_data_request(m, transmitter, supported_speeds, length=1)

                # SET_TEST_CONFIG -- The host is trying to configure our test device
                with m.State('SET_TEST_CONFIG'):
                    self.handle_register_write_request(m, self.test_config.next, self.test_config.write)

        return m

if __name__ == "__main__":    
    top_level_cli(DumbProxyApplet)
