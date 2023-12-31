/*
 * Copyright (c) 2022 Tokushima University, Japan.
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
 * Author:  Alberto Gallegos Ramonet <alramonet@is.tokushima-u.ac.jp>
 */

/*
 *      [00:01]                   [00:02]                                   [00:03]
 *  PAN Coordinator 1 (PAN: 5)       End Device                        PAN Coordinator 2 (PAN: 7)
 *       |--------100 m----------------|----------106 m -----------------------|
 *  Channel 12               (Active Scan channels 11-14)                 Channel 14
 *
 *
 * This example demonstrate the usage of the MAC MLME-SCAN.request (ACTIVE scan) primitive as
 * described by IEEE 802.15.4-2011.
 * At the beginning of the simulation, PAN coordinators are set to
 * non-beacon enabled mode and wait for any beacon requests.
 *
 * The end device initiate an Active scan where a beacon request command is transmitted on
 * on each channel. If a beacon coordinator is present and in range in the channel, it responds with
 * a beacon which contains the PAN descriptor with useful information for the association process
 * (channel number, Pan ID, coord address, link quality indicator).
 *
 * LQI range: 0 - 255
 * Where 255 is the MAX possible value used to described how clearly the packet was heard.
 * Typically, a value below 127 is considered a link with poor quality.
 */

#include <ns3/constant-position-mobility-model.h>
#include <ns3/core-module.h>
#include <ns3/log.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/packet.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>

#include <iostream>

using namespace ns3;

static void ScanConfirm(Ptr<LrWpanNetDevice> device, MlmeScanConfirmParams params) {
    std::cout << Simulator::Now().As(Time::S) << " Node " << device->GetNode()->GetId()
            << " [" << device->GetMac()->GetShortAddress() << " | "
            << device->GetMac()->GetExtendedAddress() << "]"
            << " Completed Channel Scan. "
            << "\n";

    if (params.m_status == MLMESCAN_SUCCESS) {
        std::cout << Simulator::Now().As(Time::S) << "| Enhanced active scan status SUCCESSFUL (Completed)"
                  << "\n";
        if (params.m_panDescList.size() > 0) {
            std::cout << "Device [" << device->GetMac()->GetShortAddress()
                      << "] found the following PANs:\n";
            for (long unsigned int i = 0; i < params.m_panDescList.size(); i++) {
                std::cout << "PAN DESCRIPTOR " << i << ":\n"
                          << "Pan Id: " << params.m_panDescList[i].m_coorPanId
                          << "\nChannel: " << static_cast<uint32_t>(params.m_panDescList[i].m_logCh)
                          << "\nLQI: "
                          << static_cast<uint32_t>(params.m_panDescList[i].m_linkQuality)
                          << "\nCoordinator Short Address: "
                          << params.m_panDescList[i].m_coorShortAddr 
                          << "\n DSME Superframe Spec: "
                          << params.m_panDescList[i].m_dsmeSuperframeSpec
                          << "\n Beacon Bitmap: "
                          << params.m_panDescList[i].m_bcnBitmap
                          << "\n Channel Hopping: "
                          << params.m_panDescList[i].m_channelHoppingSpec
                          << "\n\n";
            }

        } else {
            std::cout << "No PANs found (Could not find any beacons)\n";
        }

    } else {
        std::cout << "Something went wrong, scan could not be completed\n";
    }
}

int main(int argc, char* argv[]) {
    LogComponentEnableAll(LogLevel(LOG_PREFIX_TIME | LOG_PREFIX_FUNC));
    LogComponentEnable("LrWpanMac", LOG_LEVEL_INFO);
    
    // Create 2 PAN coordinator nodes, and 1 end device
    Ptr<Node> coord1 = CreateObject<Node>();
    Ptr<Node> endNode = CreateObject<Node>();
    Ptr<Node> coord2 = CreateObject<Node>();

    Ptr<LrWpanNetDevice> coord1NetDevice = CreateObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> endNodeNetDevice = CreateObject<LrWpanNetDevice>();
    Ptr<LrWpanNetDevice> coord2NetDevice = CreateObject<LrWpanNetDevice>();

    LrWpanHelper lrWpanHelper;
    
    lrWpanHelper.EnablePcap(std::string("dsme-active-enhanced-scan-filter-EB-coord1NetDevice.pcap"), coord1NetDevice, true, true);
    lrWpanHelper.EnablePcap(std::string("dsme-active-enhanced-scan-filter-EB-endNodeNetDevice.pcap"), endNodeNetDevice, true, true);
    lrWpanHelper.EnablePcap(std::string("dsme-active-enhanced-scan-filter-EB-coord2NetDevice.pcap"), coord2NetDevice, true, true);

    coord1NetDevice->SetAddress(Mac16Address("00:01"));
    endNodeNetDevice->SetAddress(Mac16Address("00:02"));
    coord2NetDevice->SetAddress(Mac16Address("00:03"));

    // Configure Spectrum channel
    Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> propModel =
        CreateObject<LogDistancePropagationLossModel>();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();
    channel->AddPropagationLossModel(propModel);
    channel->SetPropagationDelayModel(delayModel);

    coord1NetDevice->SetChannel(channel);
    endNodeNetDevice->SetChannel(channel);
    coord2NetDevice->SetChannel(channel);

    coord1->AddDevice(coord1NetDevice);
    endNode->AddDevice(endNodeNetDevice);
    coord2->AddDevice(coord2NetDevice);

    // Mobility
    Ptr<ConstantPositionMobilityModel> coord1Mobility =
        CreateObject<ConstantPositionMobilityModel>();
    coord1Mobility->SetPosition(Vector(0, 0, 0));
    coord1NetDevice->GetPhy()->SetMobility(coord1Mobility);

    Ptr<ConstantPositionMobilityModel> endNodeMobility =
        CreateObject<ConstantPositionMobilityModel>();
    endNodeMobility->SetPosition(Vector(100, 0, 0));
    endNodeNetDevice->GetPhy()->SetMobility(endNodeMobility);

    Ptr<ConstantPositionMobilityModel> coord2Mobility =
        CreateObject<ConstantPositionMobilityModel>();
    coord2Mobility->SetPosition(Vector(200, 0, 0));
    coord2NetDevice->GetPhy()->SetMobility(coord2Mobility);

    // MAC layer Callbacks hooks
    endNodeNetDevice->GetMac()->SetMlmeScanConfirmCallback(
        MakeBoundCallback(&ScanConfirm, endNodeNetDevice));

    // DSME
    coord1NetDevice->GetMac()->SetDsmeModeEnabled();
    coord2NetDevice->GetMac()->SetDsmeModeEnabled();
    coord2NetDevice->GetMac()->SetAssociateNotPermit();
    endNodeNetDevice->GetMac()->SetDsmeModeEnabled();

    //////////////////////////
    // ENHANCED ACTIVE SCAN //
    //////////////////////////

    // PAN coordinator N0 (PAN 5) is set to channel 12 in non-beacon mode but answer to beacon
    // requests.
    MlmeStartRequestParams params;
    params.m_panCoor = true;
    params.m_PanId = 5;
    params.m_bcnOrd = 15;
    params.m_sfrmOrd = 10;
    params.m_logCh = 12;

    // Beacon Bitmap
    BeaconBitmap bitmap(0, 1 << (14 - 10));
    bitmap.SetSDBitmap(0);                  // SD = 0 目前占用
    params.m_bcnBitmap = bitmap;

    // DSME
    params.m_dsmeSuperframeSpec.SetMultiSuperframeOrder(12);
    params.m_dsmeSuperframeSpec.SetChannelDiversityMode(1);
    params.m_dsmeSuperframeSpec.SetCAPReductionFlag(false);

    // Hopping Descriptor
    HoppingDescriptor hoppingDescriptor;
    hoppingDescriptor.m_HoppingSequenceID = 0x00;
    hoppingDescriptor.m_hoppingSeqLen = 0;
    hoppingDescriptor.m_channelOfs = 1;
    hoppingDescriptor.m_channelOfsBitmapLen = 16;
    hoppingDescriptor.m_channelOfsBitmap.resize(1, 2);    // offset = 1 目前占用

    params.m_hoppingDescriptor = hoppingDescriptor;

    Simulator::ScheduleWithContext(1,
                                   Seconds(1.0),
                                   &LrWpanMac::MlmeStartRequest,
                                   coord1NetDevice->GetMac(),
                                   params);

    // PAN coordinator N2 (PAN 7) is set to channel 14 in non-beacon mode but answer to beacon
    // requests.
    MlmeStartRequestParams params2;
    params2.m_panCoor = true;
    params2.m_PanId = 7;
    params2.m_bcnOrd = 15;
    params2.m_sfrmOrd = 4;
    params2.m_logCh = 14;

    // Beacon Bitmap
    BeaconBitmap bitmap2(0, 1 << (12 - 4));
    bitmap2.SetSDBitmap(0);                  // SD = 0 目前占用
    params2.m_bcnBitmap = bitmap2;

    // // DSME
    params2.m_dsmeSuperframeSpec.SetMultiSuperframeOrder(10);
    params2.m_dsmeSuperframeSpec.SetChannelDiversityMode(1);
    params2.m_dsmeSuperframeSpec.SetCAPReductionFlag(false);

    // Hopping Descriptor
    HoppingDescriptor hoppingDescriptor2;
    hoppingDescriptor2.m_HoppingSequenceID = 0x00;
    hoppingDescriptor2.m_hoppingSeqLen = 0;
    hoppingDescriptor2.m_channelOfs = 1;
    hoppingDescriptor2.m_channelOfsBitmapLen = 16;
    hoppingDescriptor2.m_channelOfsBitmap.resize(1, 2);    // offset = 1 目前占用

    params2.m_hoppingDescriptor = hoppingDescriptor2;

    Simulator::ScheduleWithContext(1,
                                   Seconds(1.0),
                                   &LrWpanMac::MlmeStartRequest,
                                   coord2NetDevice->GetMac(),
                                   params2);

    // End device N1 broadcast a single BEACON REQUEST for each channel (11, 12, 13, and 14).
    // If a coordinator is present and in range, it will respond with a beacon broadcast.
    // Scan Channels are represented by bits 0-26  (27 LSB)
    //                       ch 14  ch 11
    //                           |  |
    // 0x7800  = 0000000000000000111100000000000
    MlmeScanRequestParams scanParams;
    scanParams.m_scanType = MLMESCAN_ENHANCED_ACTIVE_SCAN;
    scanParams.m_scanChannels = 0x7800;
    scanParams.m_scanDuration = 14;
    scanParams.m_chPage = 0;
    scanParams.m_linkQualityScan = false;
    scanParams.m_frameCtrlOptions.resize(3);
    scanParams.m_frameCtrlOptions[0] = false;    // PAN_ID_SUPPRESSED
    scanParams.m_frameCtrlOptions[1] = true;    // IES_INCLUDED
    scanParams.m_frameCtrlOptions[2] = false;    // SEQ_#_SUPPRESSED

    Simulator::ScheduleWithContext(1,
                                   Seconds(3.0),
                                   &LrWpanMac::MlmeScanRequest,
                                   endNodeNetDevice->GetMac(),
                                   scanParams);

    Simulator::Stop(Seconds(1500));
    Simulator::Run();

    Simulator::Destroy();
    return 0;
}
