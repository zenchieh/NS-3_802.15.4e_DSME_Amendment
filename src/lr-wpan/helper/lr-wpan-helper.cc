/*
 * Copyright (c) 2011 The Boeing Company
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors:
 *  Gary Pei <guangyu.pei@boeing.com>
 *  Tom Henderson <thomas.r.henderson@boeing.com>
 */
#include "lr-wpan-helper.h"

#include "ns3/names.h"
#include <ns3/log.h>
#include <ns3/lr-wpan-csmaca.h>
#include <ns3/lr-wpan-error-model.h>
#include <ns3/lr-wpan-net-device.h>
#include <ns3/mobility-model.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/single-model-spectrum-channel.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LrWpanHelper");

/**
 * @brief Output an ascii line representing the Transmit event (with context)
 * @param stream the output stream
 * @param context the context
 * @param p the packet
 */
static void
AsciiLrWpanMacTransmitSinkWithContext(Ptr<OutputStreamWrapper> stream,
                                      std::string context,
                                      Ptr<const Packet> p)
{
    *stream->GetStream() << "t " << Simulator::Now().As(Time::S) << " " << context << " " << *p
                         << std::endl;
}

/**
 * @brief Output an ascii line representing the Transmit event (without context)
 * @param stream the output stream
 * @param p the packet
 */
static void
AsciiLrWpanMacTransmitSinkWithoutContext(Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
    *stream->GetStream() << "t " << Simulator::Now().As(Time::S) << " " << *p << std::endl;
}

LrWpanHelper::LrWpanHelper()
{
    m_channel = CreateObject<SingleModelSpectrumChannel>();

    Ptr<LogDistancePropagationLossModel> lossModel =
        CreateObject<LogDistancePropagationLossModel>();
    m_channel->AddPropagationLossModel(lossModel);

    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();
    m_channel->SetPropagationDelayModel(delayModel);
}

LrWpanHelper::LrWpanHelper(bool useMultiModelSpectrumChannel)
{
    if (useMultiModelSpectrumChannel)
    {
        m_channel = CreateObject<MultiModelSpectrumChannel>();
    }
    else
    {
        m_channel = CreateObject<SingleModelSpectrumChannel>();
    }
    Ptr<LogDistancePropagationLossModel> lossModel =
        CreateObject<LogDistancePropagationLossModel>();
    m_channel->AddPropagationLossModel(lossModel);

    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();
    m_channel->SetPropagationDelayModel(delayModel);
}

LrWpanHelper::~LrWpanHelper()
{
    m_channel->Dispose();
    m_channel = nullptr;
}

void
LrWpanHelper::EnableLogComponents()
{
    LogComponentEnableAll(LOG_PREFIX_TIME);
    LogComponentEnableAll(LOG_PREFIX_FUNC);
    LogComponentEnable("LrWpanCsmaCa", LOG_LEVEL_ALL);
    LogComponentEnable("LrWpanErrorModel", LOG_LEVEL_ALL);
    LogComponentEnable("LrWpanInterferenceHelper", LOG_LEVEL_ALL);
    LogComponentEnable("LrWpanMac", LOG_LEVEL_ALL);
    LogComponentEnable("LrWpanNetDevice", LOG_LEVEL_ALL);
    LogComponentEnable("LrWpanPhy", LOG_LEVEL_ALL);
    LogComponentEnable("LrWpanSpectrumSignalParameters", LOG_LEVEL_ALL);
    LogComponentEnable("LrWpanSpectrumValueHelper", LOG_LEVEL_ALL);
}

std::string
LrWpanHelper::LrWpanPhyEnumerationPrinter(LrWpanPhyEnumeration e)
{
    switch (e)
    {
    case IEEE_802_15_4_PHY_BUSY:
        return std::string("BUSY");
    case IEEE_802_15_4_PHY_BUSY_RX:
        return std::string("BUSY_RX");
    case IEEE_802_15_4_PHY_BUSY_TX:
        return std::string("BUSY_TX");
    case IEEE_802_15_4_PHY_FORCE_TRX_OFF:
        return std::string("FORCE_TRX_OFF");
    case IEEE_802_15_4_PHY_IDLE:
        return std::string("IDLE");
    case IEEE_802_15_4_PHY_INVALID_PARAMETER:
        return std::string("INVALID_PARAMETER");
    case IEEE_802_15_4_PHY_RX_ON:
        return std::string("RX_ON");
    case IEEE_802_15_4_PHY_SUCCESS:
        return std::string("SUCCESS");
    case IEEE_802_15_4_PHY_TRX_OFF:
        return std::string("TRX_OFF");
    case IEEE_802_15_4_PHY_TX_ON:
        return std::string("TX_ON");
    case IEEE_802_15_4_PHY_UNSUPPORTED_ATTRIBUTE:
        return std::string("UNSUPPORTED_ATTRIBUTE");
    case IEEE_802_15_4_PHY_READ_ONLY:
        return std::string("READ_ONLY");
    case IEEE_802_15_4_PHY_UNSPECIFIED:
        return std::string("UNSPECIFIED");
    default:
        return std::string("INVALID");
    }
}

std::string
LrWpanHelper::LrWpanMacStatePrinter(LrWpanMacState e)
{
    switch (e)
    {
    case MAC_IDLE:
        return std::string("MAC_IDLE");
    case CHANNEL_ACCESS_FAILURE:
        return std::string("CHANNEL_ACCESS_FAILURE");
    case CHANNEL_IDLE:
        return std::string("CHANNEL_IDLE");
    case SET_PHY_TX_ON:
        return std::string("SET_PHY_TX_ON");
    default:
        return std::string("INVALID");
    }
}

void
LrWpanHelper::AddMobility(Ptr<LrWpanPhy> phy, Ptr<MobilityModel> m)
{
    phy->SetMobility(m);
}

NetDeviceContainer
LrWpanHelper::Install(NodeContainer c)
{
    NetDeviceContainer devices;
    for (NodeContainer::Iterator i = c.Begin(); i != c.End(); i++)
    {
        Ptr<Node> node = *i;

        Ptr<LrWpanNetDevice> netDevice = CreateObject<LrWpanNetDevice>();
        netDevice->SetChannel(m_channel);
        node->AddDevice(netDevice);
        netDevice->SetNode(node);
        // \todo add the capability to change short address, extended
        // address and panId. Right now they are hardcoded in LrWpanMac::LrWpanMac ()
        devices.Add(netDevice);
    }
    return devices;
}

Ptr<SpectrumChannel>
LrWpanHelper::GetChannel()
{
    return m_channel;
}

void
LrWpanHelper::SetChannel(Ptr<SpectrumChannel> channel)
{
    m_channel = channel;
}

void
LrWpanHelper::SetChannel(std::string channelName)
{
    Ptr<SpectrumChannel> channel = Names::Find<SpectrumChannel>(channelName);
    m_channel = channel;
}

int64_t
LrWpanHelper::AssignStreams(NetDeviceContainer c, int64_t stream)
{
    int64_t currentStream = stream;
    Ptr<NetDevice> netDevice;
    for (NetDeviceContainer::Iterator i = c.Begin(); i != c.End(); ++i)
    {
        netDevice = (*i);
        Ptr<LrWpanNetDevice> lrwpan = DynamicCast<LrWpanNetDevice>(netDevice);
        if (lrwpan)
        {
            currentStream += lrwpan->AssignStreams(currentStream);
        }
    }
    return (currentStream - stream);
}

void LrWpanHelper::ConfigureSlotframeAllToPan(NetDeviceContainer devs, int empty_timeslots) {

}

void LrWpanHelper::AddGtsInCfp(NetDeviceContainer nodes, uint8_t numSlot, uint16_t superframeID, uint8_t slotID) {
    macDSMEACTEntity entity;
}

void LrWpanHelper::AddGtsInCfp(Ptr<NetDevice> dev
                                , bool rx
                                , uint8_t numSlot
                                , uint16_t superframeID
                                , uint8_t slotID) {
    macDSMEACTEntity entity;

    entity.m_superframeID = superframeID;
    entity.m_slotID = slotID;
    entity.m_numSlot = numSlot;
    entity.m_direction = rx;
    entity.m_type = 0x00;
    entity.m_prioritizedChAccess = 1;

    // if (entity.m_direction) {
    //     entity.m_srcAddr = receivedMacHdr.GetShortSrcAddr();
    // } else {
    //     entity.m_dstAddr = receivedMacHdr.GetShortSrcAddr();
    // }

    entity.m_cnt = 0;

    dev->GetObject<LrWpanNetDevice>()->GetMac()->AddDsmeACTEntity(superframeID, entity);
}

void LrWpanHelper::AddGtsInCfp(Ptr<NetDevice> dev
                                , bool rx
                                , uint8_t numSlot
                                , uint16_t channelOfs
                                , uint16_t superframeID
                                , uint8_t slotID) {
    macDSMEACTEntity entity;

    entity.m_superframeID = superframeID;
    entity.m_slotID = slotID;
    entity.m_numSlot = numSlot;
    entity.m_channelID = channelOfs;
    entity.m_direction = rx;
    entity.m_type = 0x00;
    entity.m_prioritizedChAccess = 1;

    if (rx) {
        entity.m_srcAddr = dev->GetObject<LrWpanNetDevice>()->GetMac()->GetShortAddress();
    } else {
        entity.m_dstAddr = dev->GetObject<LrWpanNetDevice>()->GetMac()->GetShortAddress();
    }

    entity.m_cnt = 0;

    dev->GetObject<LrWpanNetDevice>()->GetMac()->AddDsmeACTEntity(superframeID, entity);
}

void LrWpanHelper::AddGtsInCfp(Ptr<NetDevice> dev
                                , Ptr<NetDevice> dev2
                                , bool rx
                                , uint8_t numSlot
                                , uint16_t channelOfs
                                , uint16_t superframeID
                                , uint8_t slotID) {
    macDSMEACTEntity entity;

    entity.m_superframeID = superframeID;
    entity.m_slotID = slotID;
    entity.m_numSlot = numSlot;
    entity.m_channelID = channelOfs;
    entity.m_direction = rx;
    entity.m_type = 0x00;
    entity.m_prioritizedChAccess = 1;

    if (rx) {
        entity.m_srcAddr = dev2->GetObject<LrWpanNetDevice>()->GetMac()->GetShortAddress();
    } else {
        entity.m_dstAddr = dev2->GetObject<LrWpanNetDevice>()->GetMac()->GetShortAddress();
    }

    entity.m_cnt = 0;

    dev->GetObject<LrWpanNetDevice>()->GetMac()->AddDsmeACTEntity(superframeID, entity);                                
} 

void LrWpanHelper::GenerateTraffic(Ptr<NetDevice> dev, Address dst, int packet_size, double start, double duration, double interval) {
    double end = start + duration;

    Simulator::Schedule(Seconds(start), &LrWpanHelper::SendPacket, this, dev, dst, packet_size, interval, end);
}

void LrWpanHelper::SendPacket(Ptr<NetDevice> dev, Address dst, int packet_size, double interval, double end) {
    NS_LOG_DEBUG("Sending Packet");

    if (Simulator::Now().GetSeconds() <= end) {
        Ptr<Packet> pkt = Create<Packet> (packet_size);
        // dev->Send(pkt, dst, 0x86DD);

        Ptr<LrWpanNetDevice> device = DynamicCast<LrWpanNetDevice>(dev);
        device->SendInGts(pkt, dst, 0x86DD);
    }

    if (Simulator::Now().GetSeconds() <= (end + interval)) {
        Simulator::Schedule(Seconds(interval), &LrWpanHelper::SendPacket, this, dev, dst, packet_size, interval, end);
    }
}

void
LrWpanHelper::AssociateToPan(NetDeviceContainer c, uint16_t panId)
{
    NetDeviceContainer devices;
    uint16_t id = 1;
    uint8_t idBuf[2];

    for (NetDeviceContainer::Iterator i = c.Begin(); i != c.End(); i++)
    {
        Ptr<LrWpanNetDevice> device = DynamicCast<LrWpanNetDevice>(*i);
        if (device)
        {
            idBuf[0] = (id >> 8) & 0xff;
            idBuf[1] = (id >> 0) & 0xff;
            Mac16Address address;
            address.CopyFrom(idBuf);

            device->GetMac()->SetPanId(panId);
            device->GetMac()->SetShortAddress(address);
            id++;
        }
    }
}

void
LrWpanHelper::AssociateToBeaconPan(NetDeviceContainer c,
                                   uint16_t panId,
                                   Mac16Address coor,
                                   uint8_t bcnOrd,
                                   uint8_t sfrmOrd)
{
    NetDeviceContainer devices;
    uint16_t id = 1;
    uint8_t idBuf[2];
    Mac16Address address;

    if (bcnOrd > 14)
    {
        NS_LOG_DEBUG("The Beacon Order must be an int between 0 and 14");
        return;
    }

    if ((sfrmOrd > 14) || (sfrmOrd > bcnOrd))
    {
        NS_LOG_DEBUG("The Superframe Order must be an int between 0 and 14, and less or equal to "
                     "Beacon Order");
        return;
    }

    for (NetDeviceContainer::Iterator i = c.Begin(); i != c.End(); i++)
    {
        Ptr<LrWpanNetDevice> device = DynamicCast<LrWpanNetDevice>(*i);
        if (device)
    {
            idBuf[0] = (id >> 8) & 0xff;
            idBuf[1] = (id >> 0) & 0xff;
            address.CopyFrom(idBuf);

            device->GetMac()->SetShortAddress(address);

            if (address == coor)
            {
                MlmeStartRequestParams params;
                params.m_panCoor = true;
                params.m_PanId = panId;
                params.m_bcnOrd = bcnOrd;
                params.m_sfrmOrd = sfrmOrd;

                Ptr<UniformRandomVariable> uniformRandomVariable =
                    CreateObject<UniformRandomVariable>();
                ;
                Time jitter = Time(MilliSeconds(uniformRandomVariable->GetInteger(0, 10)));

                Simulator::Schedule(jitter, &LrWpanMac::MlmeStartRequest, device->GetMac(), params);
            }
            else
            {
                device->GetMac()->SetPanId(panId);
                device->GetMac()->SetAssociatedCoor(coor);
            }
            id++;
        }
    }
}

void LrWpanHelper::AssociateToBeaconPan(NetDeviceContainer c,
                              Mac16Address coor,
                              MlmeStartRequestParams params) {

    NetDeviceContainer devices;
    uint16_t id = 1;
    uint8_t idBuf[2];
    Mac16Address address;

    for (NetDeviceContainer::Iterator i = c.Begin(); i != c.End(); i++) {
        Ptr<LrWpanNetDevice> device = DynamicCast<LrWpanNetDevice>(*i);

        if (device) {
            idBuf[0] = (id >> 8) & 0xff;
            idBuf[1] = (id >> 0) & 0xff;
            address.CopyFrom(idBuf);

            device->GetMac()->SetShortAddress(address);

            if (address == coor) {
                Ptr<UniformRandomVariable> uniformRandomVariable =
                    CreateObject<UniformRandomVariable>();
                ;
                // Time jitter = Time(MilliSeconds(uniformRandomVariable->GetInteger(0, 10)));

                // Simulator::Schedule(jitter, &LrWpanMac::MlmeStartRequest, device->GetMac(), params);

                Simulator::ScheduleNow(&LrWpanMac::MlmeStartRequest, device->GetMac(), params);
                
            } else {
                device->GetMac()->SetPanId(params.m_PanId);
                device->GetMac()->SetAssociatedCoor(coor);
            }

            id++;
        }
    }                            
}

void LrWpanHelper::CoordBoostrap(Ptr<NetDevice> dev, PanDescriptor descriptor, uint16_t sdIndex, MlmeStartRequestParams params) {
    Ptr<LrWpanNetDevice> device = DynamicCast<LrWpanNetDevice>(dev);

    device->GetMac()->AddPanDescriptor(descriptor);
    device->GetMac()->SetDescIndexOfAssociatedPan(0);
    device->GetMac()->SetTimeSlotToSendBcn(sdIndex);

    Ptr<UniformRandomVariable> uniformRandomVariable =
                    CreateObject<UniformRandomVariable>();
    Time jitter = Time(MilliSeconds(uniformRandomVariable->GetInteger(10, 20)));       

    // Simulator::Schedule(jitter, &LrWpanMac::SetAsCoordinator, device->GetMac());
    // Simulator::Schedule(jitter, &LrWpanMac::MlmeStartRequest, device->GetMac(), params);

    Simulator::ScheduleNow(&LrWpanMac::SetAsCoordinator, device->GetMac());
    Simulator::ScheduleNow(&LrWpanMac::MlmeStartRequest, device->GetMac(), params);
}

/**
 * @brief Write a packet in a PCAP file
 * @param file the output file
 * @param packet the packet
 */
static void
PcapSniffLrWpan(Ptr<PcapFileWrapper> file, Ptr<const Packet> packet)
{
    file->Write(Simulator::Now(), packet);
}

void
LrWpanHelper::EnablePcapInternal(std::string prefix,
                                 Ptr<NetDevice> nd,
                                 bool promiscuous,
                                 bool explicitFilename)
{
    NS_LOG_FUNCTION(this << prefix << nd << promiscuous << explicitFilename);
    //
    // All of the Pcap enable functions vector through here including the ones
    // that are wandering through all of devices on perhaps all of the nodes in
    // the system.
    //

    // In the future, if we create different NetDevice types, we will
    // have to switch on each type below and insert into the right
    // NetDevice type
    //
    Ptr<LrWpanNetDevice> device = nd->GetObject<LrWpanNetDevice>();
    if (!device)
    {
        NS_LOG_INFO("LrWpanHelper::EnablePcapInternal(): Device "
                    << device << " not of type ns3::LrWpanNetDevice");
        return;
    }

    PcapHelper pcapHelper;

    std::string filename;
    if (explicitFilename)
    {
        filename = prefix;
    }
    else
    {
        filename = pcapHelper.GetFilenameFromDevice(prefix, device);
    }

    Ptr<PcapFileWrapper> file =
        pcapHelper.CreateFile(filename, std::ios::out, PcapHelper::DLT_IEEE802_15_4);

    if (promiscuous == true)
    {
        device->GetMac()->TraceConnectWithoutContext("PromiscSniffer",
                                                     MakeBoundCallback(&PcapSniffLrWpan, file));
    }
    else
    {
        device->GetMac()->TraceConnectWithoutContext("Sniffer",
                                                     MakeBoundCallback(&PcapSniffLrWpan, file));
    }
}

void
LrWpanHelper::EnableAsciiInternal(Ptr<OutputStreamWrapper> stream,
                                  std::string prefix,
                                  Ptr<NetDevice> nd,
                                  bool explicitFilename)
{
    uint32_t nodeid = nd->GetNode()->GetId();
    uint32_t deviceid = nd->GetIfIndex();
    std::ostringstream oss;

    Ptr<LrWpanNetDevice> device = nd->GetObject<LrWpanNetDevice>();
    if (!device)
    {
        NS_LOG_INFO("LrWpanHelper::EnableAsciiInternal(): Device "
                    << device << " not of type ns3::LrWpanNetDevice");
        return;
    }

    //
    // Our default trace sinks are going to use packet printing, so we have to
    // make sure that is turned on.
    //
    Packet::EnablePrinting();

    //
    // If we are not provided an OutputStreamWrapper, we are expected to create
    // one using the usual trace filename conventions and do a Hook*WithoutContext
    // since there will be one file per context and therefore the context would
    // be redundant.
    //
    if (!stream)
    {
        //
        // Set up an output stream object to deal with private ofstream copy
        // constructor and lifetime issues.  Let the helper decide the actual
        // name of the file given the prefix.
        //
        AsciiTraceHelper asciiTraceHelper;

        std::string filename;
        if (explicitFilename)
        {
            filename = prefix;
        }
        else
        {
            filename = asciiTraceHelper.GetFilenameFromDevice(prefix, device);
        }

        Ptr<OutputStreamWrapper> theStream = asciiTraceHelper.CreateFileStream(filename);

        // Ascii traces typically have "+", '-", "d", "r", and sometimes "t"
        // The Mac and Phy objects have the trace sources for these
        //

        asciiTraceHelper.HookDefaultReceiveSinkWithoutContext<LrWpanMac>(device->GetMac(),
                                                                         "MacRx",
                                                                         theStream);

        device->GetMac()->TraceConnectWithoutContext(
            "MacTx",
            MakeBoundCallback(&AsciiLrWpanMacTransmitSinkWithoutContext, theStream));

        asciiTraceHelper.HookDefaultEnqueueSinkWithoutContext<LrWpanMac>(device->GetMac(),
                                                                         "MacTxEnqueue",
                                                                         theStream);
        asciiTraceHelper.HookDefaultDequeueSinkWithoutContext<LrWpanMac>(device->GetMac(),
                                                                         "MacTxDequeue",
                                                                         theStream);
        asciiTraceHelper.HookDefaultDropSinkWithoutContext<LrWpanMac>(device->GetMac(),
                                                                      "MacTxDrop",
                                                                      theStream);

        return;
    }

    //
    // If we are provided an OutputStreamWrapper, we are expected to use it, and
    // to provide a context.  We are free to come up with our own context if we
    // want, and use the AsciiTraceHelper Hook*WithContext functions, but for
    // compatibility and simplicity, we just use Config::Connect and let it deal
    // with the context.
    //
    // Note that we are going to use the default trace sinks provided by the
    // ascii trace helper.  There is actually no AsciiTraceHelper in sight here,
    // but the default trace sinks are actually publicly available static
    // functions that are always there waiting for just such a case.
    //

    oss.str("");
    oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid
        << "/$ns3::LrWpanNetDevice/Mac/MacRx";
    device->GetMac()->TraceConnect(
        "MacRx",
        oss.str(),
        MakeBoundCallback(&AsciiTraceHelper::DefaultReceiveSinkWithContext, stream));

    oss.str("");
    oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid
        << "/$ns3::LrWpanNetDevice/Mac/MacTx";
    device->GetMac()->TraceConnect(
        "MacTx",
        oss.str(),
        MakeBoundCallback(&AsciiLrWpanMacTransmitSinkWithContext, stream));

    oss.str("");
    oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid
        << "/$ns3::LrWpanNetDevice/Mac/MacTxEnqueue";
    device->GetMac()->TraceConnect(
        "MacTxEnqueue",
        oss.str(),
        MakeBoundCallback(&AsciiTraceHelper::DefaultEnqueueSinkWithContext, stream));

    oss.str("");
    oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid
        << "/$ns3::LrWpanNetDevice/Mac/MacTxDequeue";
    device->GetMac()->TraceConnect(
        "MacTxDequeue",
        oss.str(),
        MakeBoundCallback(&AsciiTraceHelper::DefaultDequeueSinkWithContext, stream));

    oss.str("");
    oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid
        << "/$ns3::LrWpanNetDevice/Mac/MacTxDrop";
    device->GetMac()->TraceConnect(
        "MacTxDrop",
        oss.str(),
        MakeBoundCallback(&AsciiTraceHelper::DefaultDropSinkWithContext, stream));
}

} // namespace ns3
