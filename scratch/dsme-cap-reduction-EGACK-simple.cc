/*
 * Copyright (c) 2020 Ritsumeikan University, Shiga, Japan
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
 * Author: Alberto Gallegos Ramonet <ramonet@fc.ritsumei.ac.jp>
 */

#include "ns3/core-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/propagation-module.h"
#include "ns3/sixlowpan-module.h"
#include "ns3/spectrum-module.h"
#include <fstream>

using namespace ns3;

#define BO 6
#define SO 3
#define MO 5

#define NUM_COORD 2 // The number of coord, PAN-C need to be included.
#define NUM_RFD 1 // The number of RFD.

#define BIT(X) (1 << 2^X)

static double pktRecv = 0;
static double pktSent = 0;

typedef enum
{
    CHANNEL_ADAPTATION = 0,
    CHANNEL_HOPPING = 1
} LrWpanDsmeChannelDiversity;

static void
dataSentMacConfirm(McpsDataConfirmParams params) // McpsDataConfirmCallBack
{
    // In the case of transmissions with the Ack flag activated, the transaction is only
    // successful if the Ack was received.
    if (params.m_status == LrWpanMcpsDataConfirmStatus::IEEE_802_15_4_SUCCESS)
    {
        NS_LOG_UNCOND("**********" << Simulator::Now().As(Time::S)
                                   << " | Transmission successfully sent");
        pktSent += 1;
    }
}

static void dataIndication(McpsDataIndicationParams params, Ptr<Packet> p) {
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << " secs | The RX higher layer received DATA packet of size " << p->GetSize() << " bytes");
    pktRecv += 1;
}

int main(int argc, char** argv) {
    bool verbose = true;

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "turn on log components", verbose);
    cmd.Parse(argc, argv);

    if (verbose) {
        LogComponentEnableAll(LOG_PREFIX_TIME);
        LogComponentEnableAll(LOG_PREFIX_FUNC);
        LogComponentEnable("LrWpanMac", LOG_LEVEL_INFO);
        // LogComponentEnable("LrWpanPhy", LOG_LEVEL_INFO);
        // LogComponentEnable("LrWpanCsmaCa", LOG_LEVEL_INFO);
        LogComponentEnable("LrWpanHelper", LOG_LEVEL_ALL);
        LogComponentEnable("Ping6Application", LOG_LEVEL_INFO);
    }
    
    NodeContainer nodes;
    nodes.Create(NUM_COORD + NUM_RFD);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    mobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                  "X",
                                  StringValue("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"),
                                  "Y",
                                  StringValue("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"));

    mobility.Install(nodes);

    LrWpanHelper lrWpanHelper(true);
    // Add and install the LrWpanNetDevice for each node
    NetDeviceContainer lrwpanDevices = lrWpanHelper.Install(nodes);

    uint16_t panChannelOfs = 0;
    
    std::vector<uint16_t> channelOffsets;
    // Setting channel offset array
    for(int i = 0; i < 16; i++)
    {
        channelOffsets.push_back(i);
    }

    uint16_t numOfChannelsSupported = 6;

    // // In this example, Hopping Sequence is {1, 2, 3, 4, 5, 6}
    // std::vector<uint16_t> hoppingSequence;
    // for(int i = 0; i < numOfChannelsSupported; i++)
    // {
    //     hoppingSequence[i] = i + 1;
    // }

    // callback hook
    McpsDataConfirmCallback cb1;
    cb1 = MakeCallback(&dataSentMacConfirm);
    McpsDataIndicationCallback cb2;
    cb2 = MakeCallback(&dataIndication);

    // Dsme Network Parameters
    uint16_t panId = 5;
    uint16_t bcnOrder = BO;
    uint16_t multisuperfrmOrder = MO;
    uint16_t superfrmOrder = SO;
    uint8_t channelNum = 11;
    bool capReduction = true;

    for (unsigned int i = 0; i < lrwpanDevices.GetN(); ++i) {
        Ptr<LrWpanNetDevice> dev = lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>();
        dev->GetMac()->SetMcpsDataConfirmCallback(cb1);
        dev->GetMac()->SetMcpsDataIndicationCallback(cb2);

        dev->GetMac()->SetNumOfChannelSupported(numOfChannelsSupported);
        
        // Cap Reduction setting
        dev->GetMac()->SetCAPReduction(capReduction);

        // Set the group ack policy tp self-designed enhanced group.
        dev->GetMac()->SetGroupAckPolicy(LrWpanGroupAckPolicy::GROUP_ACK_ENHANCED);
    }

    // Pan Coord mlme-start.request params
    MlmeStartRequestParams startParams;
    startParams.m_panCoor = true;
    startParams.m_PanId = panId;
    startParams.m_bcnOrd = bcnOrder;
    startParams.m_sfrmOrd = superfrmOrder;
    startParams.m_logCh = channelNum;

    BeaconBitmap bitmap(0, 1 << (bcnOrder - superfrmOrder));
    bitmap.SetSDIndex(0);                  // PAN-C beacon use SDIDx = 0 (beacon TX at SDIdx 0)
    startParams.m_bcnBitmap = bitmap;

    HoppingDescriptor hoppingDescriptor;
    hoppingDescriptor.m_HoppingSequenceID = 0x00;
    hoppingDescriptor.m_hoppingSeqLen = 0;
    hoppingDescriptor.m_channelOfs = panChannelOfs;
    hoppingDescriptor.m_channelOfsBitmapLen = 16;
    hoppingDescriptor.m_channelOfsBitmap.resize(1, BIT(panChannelOfs));   

    startParams.m_hoppingDescriptor = hoppingDescriptor;

    DsmeSuperFrameField dsmeSuperframeField;
    dsmeSuperframeField.SetMultiSuperframeOrder(multisuperfrmOrder);
    dsmeSuperframeField.SetChannelDiversityMode(CHANNEL_HOPPING);
    dsmeSuperframeField.SetCAPReductionFlag(capReduction);

    startParams.m_dsmeSuperframeSpec = dsmeSuperframeField;

    lrWpanHelper.AssociateToBeaconPan(lrwpanDevices
                                        , Mac16Address("00:01")
                                        , startParams);


    // 2nd level Coordinator setting, let other coordinator associate with pan-C
    for (unsigned int i = 1; i < NUM_COORD; ++i) {
        MlmeSyncRequestParams syncParams;
        syncParams.m_logChPage = 0; 
        syncParams.m_trackBcn = true; 
        lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->TrackCoordinatorBeacon(syncParams);

        MlmeStartRequestParams params;
        params.m_panCoor = false;
        params.m_PanId = panId;
        params.m_bcnOrd = bcnOrder;
        params.m_sfrmOrd = superfrmOrder;

        BeaconBitmap bitmap(0, 1 << (bcnOrder - superfrmOrder));
        bitmap.SetSDIndex(i);                 
        params.m_bcnBitmap = bitmap;

        HoppingDescriptor hoppingDescriptor;
        hoppingDescriptor.m_HoppingSequenceID = 0x00;
        hoppingDescriptor.m_hoppingSeqLen = 0;
        hoppingDescriptor.m_channelOfs = channelOffsets[i];
        hoppingDescriptor.m_channelOfsBitmapLen = 16;
        hoppingDescriptor.m_channelOfsBitmap.resize(1, 1 + (2 << i));   

        params.m_hoppingDescriptor = hoppingDescriptor;

        // Pan Descriptor
        PanDescriptor panDescriptor;
        panDescriptor.m_coorPanId = panId;
        panDescriptor.m_coorShortAddr = Mac16Address("00:01");
        panDescriptor.m_logCh = channelNum;

        SuperframeField superframeField;
        superframeField.SetSuperframeOrder(superfrmOrder);
        superframeField.SetBeaconOrder(bcnOrder);
        panDescriptor.m_superframeSpec = superframeField;

        panDescriptor.m_dsmeSuperframeSpec = dsmeSuperframeField;
        panDescriptor.m_bcnBitmap = bitmap;

        lrWpanHelper.CoordBoostrap(lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()
                                    , panDescriptor
                                    , i
                                    , params);
    }
    
    // GTSs setting
    for (unsigned int i = 0 ; i < lrwpanDevices.GetN(); ++i) {
        lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->SetMcpsDataReqGts(true);
    }

    for (unsigned int i = 0 ; i < lrwpanDevices.GetN(); ++i) {
        lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>()->GetMac()->ResizeScheduleGTSsEvent(bcnOrder, 
                                                                                              multisuperfrmOrder, 
                                                                                              superfrmOrder);
    }

    int pktSize = 10;
    uint16_t superframeID = 1;
    for (int i = 0; i < 1; ++i) {
        int childIdx = i + NUM_COORD;
        // Channel Offset setting
        lrwpanDevices.Get(childIdx)->GetObject<LrWpanNetDevice>()->SetChannelOffset(channelOffsets[0]);


        // Setting the GTS of Coord and devices with channel offset & TX/RX direction & SPFIdx & slotIDx etc.
        // allocate slot 0 ~ slot 2
        for(int slotIdx = 0; slotIdx < 3; slotIdx++)
        {
            lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(1)->GetObject<LrWpanNetDevice>(), true, 1, channelOffsets[0]
                                , superframeID, slotIdx);

            lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(childIdx)->GetObject<LrWpanNetDevice>(), false, 1, channelOffsets[0]
                                , superframeID, slotIdx);       
        }        

        // Setting parameters of sending the data packets.
        // This will continuously TX packet , non stop.
        // lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(childIdx), lrwpanDevices.Get(1)->GetAddress(), pktSize, 1.11553, 100000.0, 0.0001);      

        // This will send packet at slot 0 ~ slot 2 (allocated slots like above)
        lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(childIdx), lrwpanDevices.Get(1)->GetAddress(), pktSize, 1.11553, 0.005, 0.0001);  
        lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(childIdx), lrwpanDevices.Get(1)->GetAddress(), pktSize, 1.1232, 0.005, 0.0001);
        lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(childIdx), lrwpanDevices.Get(1)->GetAddress(), pktSize, 1.1308, 0.005, 0.0001);

        // Setting the GTS for Group Ack slot (at slot 6 & slot 14 for CAP reduction).
        lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(1)->GetObject<LrWpanNetDevice>(), false, 1, channelOffsets[0] // Coord for TX
                            , superframeID, 6);
        lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(childIdx)->GetObject<LrWpanNetDevice>(), true, 1, channelOffsets[0] // Devices for RX
                            , superframeID, 6);  

        lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(1)->GetObject<LrWpanNetDevice>(), false, 1, channelOffsets[0] // Coord for TX
                            , superframeID, 14);     
        lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(childIdx)->GetObject<LrWpanNetDevice>(), true, 1, channelOffsets[0] // Devices for RX
                            , superframeID, 14);    
    }

    // for (int i = 0; i < 1; ++i) {
    //     int childIdx = i + NUM_COORD;
    //     // Channel Offset setting
    //     lrwpanDevices.Get(childIdx)->GetObject<LrWpanNetDevice>()->SetChannelOffset(channelOffsets[0]);

    //     lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(1)->GetObject<LrWpanNetDevice>(), true, 1, channelOffsets[0]
    //                             , superframeID, 1);

    //     lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(childIdx)->GetObject<LrWpanNetDevice>(), false, 1, channelOffsets[0]
    //                             , superframeID, 1);

    //     lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(childIdx), lrwpanDevices.Get(1)->GetAddress(), pktSize, 1.11553, 100000.0, 0.0001);                      
    // }




    // // superframe ID = 1
    // superframeID++;

    // for (int i = 0; i < 15; ++i) {
    //     int childIdx = i + 3;

    //     // Channel Offset setting
    //     lrwpanDevices.Get(childIdx)->GetObject<LrWpanNetDevice>()->SetChannelOffset(channelOffsets[1]);

    //     // lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(1)->GetObject<LrWpanNetDevice>(), true, 1, superframeID, i);
    //     // lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(childIdx)->GetObject<LrWpanNetDevice>(), false, 1, superframeID, i);

    //     lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(1)->GetObject<LrWpanNetDevice>(), true, 1, channelOffsets[1]
    //                             , superframeID, i);

    //     lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(childIdx)->GetObject<LrWpanNetDevice>(), false, 1, channelOffsets[1]
    //                             , superframeID, i);

    //     lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(childIdx), lrwpanDevices.Get(1)->GetAddress(), pktSize, 200, 100000.0, 125.8295);
    // }

    // // superframe id = 2
    // for (int i = 0; i < 15; ++i) {
    //     int childIdx = i + 18;

    //     // Channel Offset setting
    //     lrwpanDevices.Get(childIdx)->GetObject<LrWpanNetDevice>()->SetChannelOffset(channelOffsets[2]);

    //     // lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(2)->GetObject<LrWpanNetDevice>(), true, 1, superframeID, i);
    //     // lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(childIdx)->GetObject<LrWpanNetDevice>(), false, 1, superframeID, i);

    //     lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(2)->GetObject<LrWpanNetDevice>(), true, 1, channelOffsets[2]
    //                             , superframeID, i);

    //     lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(childIdx)->GetObject<LrWpanNetDevice>(), false, 1, channelOffsets[2]
    //                             , superframeID, i);

    //     lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(childIdx), lrwpanDevices.Get(2)->GetAddress(), pktSize, 200, 100000.0, 125.8295);
    // }
    
    // // lrwpanDevices.Get(1)->GetObject<LrWpanNetDevice>()->SetChannelOffset(channelOffsets[1]);
    // // lrwpanDevices.Get(2)->GetObject<LrWpanNetDevice>()->SetChannelOffset(channelOffsets[1]);

    // // lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(1)->GetObject<LrWpanNetDevice>(), true, 1, 1, 0);
    // // lrWpanHelper.AddGtsInCfp(lrwpanDevices.Get(2)->GetObject<LrWpanNetDevice>(), false, 1, 1, 0);

    // // lrWpanHelper.GenerateTraffic(lrwpanDevices.Get(2), lrwpanDevices.Get(1)->GetAddress(), pktSize, 300, 100000.0, 125.8295);

    AsciiTraceHelper ascii;
    lrWpanHelper.EnableAsciiAll(ascii.CreateFileStream("Gack.tr"));
    lrWpanHelper.EnablePcapAll(std::string("Gack"), true);

    Simulator::Stop(Seconds(1.96607));
    // Simulator::Stop(Seconds(50));

    Simulator::Run();

    std::cout << "pktSent: " << pktSent << std::endl;
    std::cout << "pktRecv: " << pktRecv << std::endl;
    std::cout << "Delivery ratio: " << pktRecv / pktSent << std::endl;

    Simulator::Destroy();
}