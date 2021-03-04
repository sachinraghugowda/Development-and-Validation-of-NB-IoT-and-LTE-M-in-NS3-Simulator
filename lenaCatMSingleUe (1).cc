/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Author: Manuel Requena <manuel.requena@cttc.es>
 *
 *
 * lena-x2-handover.cc-> modified by: Sachin Gowda
 */


#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include "ns3/data-rate.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/radio-environment-map-helper.h"
#include "ns3/flow-monitor-module.h"
#include <vector>
#include <stdlib.h> 
#include <sstream>
#include "ns3/netanim-module.h"
#include "ns3/lte-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/config-store-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/stats-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("CatMsingleUEtoremoteHost");

 uint16_t numberOfUes = 10;                                                                                                 //sachin start

void
NotifyConnectionEstablishedEnb (std::string context,
                               uint64_t imsi,
                               uint16_t cellid,
                               uint16_t rnti
                               )
{
     if ( imsi <= numberOfUes)
             {
  std::cout << Simulator::Now ().GetSeconds () << " " << context
       << " UE IMSI is LTE-M-UE " << imsi  << ": connected to CellId " << cellid<< " with RNTI " << rnti<< std::endl;
              }
           else
            {
 std::cout << Simulator::Now ().GetSeconds () << " " << context
        << " UE IMSI is NBIOT-UE " << imsi << ": connected to CellId " << cellid << " with RNTI " << rnti<< std::endl;
            }
         
}                                                                                                                            //sachin end


/**
 * Simulation script for Cat-M single UE to remote host data uploading
 * It instantiates one eNodeB, attaches one UE to the eNB and
 * UE send data to remote host
 */

/*
void ThroughputMonitor (FlowMonitorHelper *fmhelper, Ptr<FlowMonitor> flowMon)
{
	std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats();
	Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier());
	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin (); stats != flowStats.end (); ++stats)
	{
		Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow (stats->first);
		std::cout<<"Flow ID			: " << stats->first <<" ; "<< fiveTuple.sourceAddress <<" -----> "<<fiveTuple.destinationAddress<<std::endl;
		std::cout<<"Tx Packets = " << stats->second.txPackets<<std::endl;
		std::cout<<"Tx Bytes = " << stats->second.txBytes<<std::endl;
		std::cout<<"Rx Packets = " << stats->second.rxPackets<<std::endl;
		std::cout<<"Rx Bytes = " << stats->second.rxBytes<<std::endl;
		std::cout<<"Duration		: "<<stats->second.timeLastTxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds()<<std::endl;
		std::cout<<"Last Received Packet	: "<< stats->second.timeLastTxPacket.GetSeconds()<<" Seconds"<<std::endl;
		std::cout<<"Lost Packet : "<< stats->second.lostPackets<<std::endl;
		std::cout<<"Throughput: " << stats->second.rxBytes*1.0/(stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstRxPacket.GetSeconds()) << " B/s"<<std::endl;
		std::cout<<"---------------------------------------------------------------------------"<<std::endl;
	}
	//Simulator::Schedule(Seconds(1),&ThroughputMonitor, fmhelper, flowMon);
}

Vector
GetCurrentPosition(Ptr<Node> node)
{
  Ptr<MobilityModel> MM = node -> GetObject<MobilityModel>();
  return MM -> GetPosition();
}

static void
RelocateUE(NodeContainer nodes, int pos_diff)
{
  Ptr<MobilityModel> MM;
  Vector pos;
  Ptr<Node> node;
  
  node = nodes.Get(0);
  pos = GetCurrentPosition(node);
  MM = node -> GetObject<MobilityModel>();
  MM -> SetPosition(Vector3D(pos.x+pos_diff, pos.y, pos.z));
  //VM = UEinHeNB.Get(0) -> GetObject<ConstantVelocityMobilityModel>();std::cout << "AAAAAAAAAaa" <<std::endl;
  std::cout << Simulator::Now().GetSeconds () <<": Node " << node->GetId() <<": "<< MM -> GetPosition().x <<"," << MM -> GetPosition().y <<"," << MM -> GetPosition().z << std::endl;
}*/


int
main (int argc, char *argv[])
{
  // LogLevel logLevel = (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME | LOG_LEVEL_ALL);

  // LogComponentEnable ("LteHelper", logLevel);
  // LogComponentEnable ("EpcHelper", logLevel);
  // LogComponentEnable ("EpcEnbApplication", logLevel);
  // LogComponentEnable ("EpcX2", logLevel);
  // LogComponentEnable ("EpcSgwPgwApplication", logLevel);

  // LogComponentEnable ("LteEnbRrc", logLevel);
  // LogComponentEnable ("LteEnbNetDevice", logLevel);
  // LogComponentEnable ("LteUeRrc", logLevel);
  // LogComponentEnable ("LteUeNetDevice", logLevel);

  uint16_t numberOfUes = 10;
  uint16_t numberOfNbues = 2;                                             //sachin
  uint16_t numberOfEnbs = 1;
 // uint16_t numberOfNbenbs = 1;                                            //sachin
  uint16_t numBearersPerUe = 1;
  double simTime = 2.00;
  //double distance = 10.0;
  std::string animFile = "CatMUe.xml";
  //double loc = 0;
  double interSiteDistance = 2000;
  //uint16_t n_dist = 30;
  //uint16_t t_diff = 10;
  uint16_t pos_diff = 0;
  //uint16_t pos_diff2 = 0;


  // change some default attributes so that they are reasonable for
  // this scenario, but do this before processing command line
  // arguments, so that the user is allowed to override these settings
  Config::SetDefault ("ns3::UdpClient::Interval", TimeValue (MilliSeconds (100)));
  Config::SetDefault ("ns3::UdpClient::MaxPackets", UintegerValue (1000000));
  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (false));
  Config::SetDefault ("ns3::RadioBearerStatsCalculator::EpochDuration", TimeValue (Seconds(1)));

  Config::SetDefault ("ns3::LtePhy::TxTimeInterval", DoubleValue (0.001));
  //Config::SetDefault ("ns3::LteUePhy::UeMacToChDelay", UintegerValue (3));
  Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (20));
  Config::SetDefault ("ns3::LteEnbPhy::NoiseFigure", DoubleValue (5));
  Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (23));
  Config::SetDefault ("ns3::LteUePhy::NoiseFigure", DoubleValue (9));
  Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue (6));
  Config::SetDefault ("ns3::LteEnbNetDevice::NbiotDlBandwidth", UintegerValue (1));                    //sachin
  Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue (6));
  Config::SetDefault ("ns3::LteEnbNetDevice::NbiotUlBandwidth", UintegerValue (1));                    //sachin
 Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (160));

  
  // Command line arguments
  CommandLine cmd;
  cmd.AddValue ("numberOfUes", "Number of UEs", numberOfUes);
  cmd.AddValue ("numberOfNbues", "Number of NbUEs", numberOfNbues);                                    //sachin
  cmd.AddValue ("numberOfEnbs", "Number of eNodeBs", numberOfEnbs);
  //cmd.AddValue ("numberOfNbenbs", "Number of nbeNodeBs", numberOfNbenbs);                              //sachin
  cmd.AddValue ("animFile",  "File Name for Animation Output", animFile);
  cmd.AddValue ("simTime", "Total duration of the simulation (in seconds)", simTime);
  cmd.Parse (argc, argv);


  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FriisPropagationLossModel"));
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);
  lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
  lteHelper->SetHandoverAlgorithmType ("ns3::NoOpHandoverAlgorithm"); // disable automatic handover

  //Cat-M configurations for eNB and UE PHY
  //Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (20));
  //Config::SetDefault ("ns3::LteEnbPhy::NoiseFigure", DoubleValue (5));
  //Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (20));
  //Config::SetDefault ("ns3::LteUePhy::NoiseFigure", DoubleValue (9));
  lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
  lteHelper->SetEnbDeviceAttribute ("DlEarfcn", UintegerValue (100));
  lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (100 + 18000));
  lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (6));
  lteHelper->SetEnbDeviceAttribute ("NbiotDlBandwidth", UintegerValue (1));                             //sachin
  lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (6));
  lteHelper->SetEnbDeviceAttribute ("NbiotUlBandwidth", UintegerValue (1));                             //sachin
  
  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create the Internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Mb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);


  // Routing of the Internet Host (towards the LTE network)
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  // interface 0 is localhost, 1 is the p2p device
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  NodeContainer ueNodes;
  NodeContainer nbueNodes;                                                                                     //sachin
  NodeContainer enbNodes;
 // NodeContainer nbenbNodes;                                                                                    //sachin
  enbNodes.Create (numberOfEnbs);
 // nbenbNodes.Create (numberOfNbenbs);                                                                          //sachin
  ueNodes.Create (numberOfUes);
  nbueNodes.Create (numberOfNbues);                                                                             //sachin

  // Install Mobility Model
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  for (uint16_t i = 0; i < numberOfEnbs; i++)
    {
      //positionAlloc->Add (Vector (distance * 2 * i, 0, 0));
	positionAlloc->Add (Vector (i, 0, 0));
    }
  for (uint16_t i = 0; i < numberOfUes; i++)
    {
	//positionAlloc->Add (Vector (loc, 0, 0));
	positionAlloc->Add (Vector (pos_diff, 0, 0));
    }
  positionAlloc->Add (Vector (100, 100, 0));
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (enbNodes);
  mobility.Install (ueNodes);
  mobility.Install (remoteHostContainer);

  Ptr<LteHexGridEnbTopologyHelper> lteHexGridEnbTopologyHelper = CreateObject<LteHexGridEnbTopologyHelper> ();
  lteHexGridEnbTopologyHelper->SetLteHelper (lteHelper);
	
  lteHexGridEnbTopologyHelper->SetAttribute ("InterSiteDistance", DoubleValue (interSiteDistance));
  lteHexGridEnbTopologyHelper->SetAttribute ("MinX", DoubleValue (0));//-1000));
  lteHexGridEnbTopologyHelper->SetAttribute ("MinY", DoubleValue (0));//-1732));
  lteHexGridEnbTopologyHelper->SetAttribute ("GridWidth", UintegerValue (2));
  //Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (macroEnbTxPowerDbm));
	
  lteHelper->SetEnbAntennaModelType ("ns3::ParabolicAntennaModel");
  lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (70));
  lteHelper->SetEnbAntennaModelAttribute ("MaxAttenuation",     DoubleValue (20.0));

  // Install LTE Devices in eNB and UEs
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
 // NetDeviceContainer nbenbLteDevs = lteHelper->InstallNbEnbDevice (nbenbNodes);                               //sachin 
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);
  NetDeviceContainer nbueLteDevs = lteHelper->InstallNbUeDevice (nbueNodes);                                  //sachin

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  internet.Install (nbueNodes);                                                                               //sachin
  Ipv4InterfaceContainer ueIpIfaces;
  Ipv4InterfaceContainer nbueIpIfaces;                                                                        //sachin
  ueIpIfaces = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));
  nbueIpIfaces = epcHelper->AssignNbueIpv4Address (NetDeviceContainer (nbueLteDevs));                           //sachin



  // Attach all UEs to the first eNodeB
  for (uint16_t i = 0; i < numberOfUes; i++)
    {
      lteHelper->Attach (ueLteDevs.Get (i), enbLteDevs.Get (0));
    }


  for (uint16_t i = 0; i < numberOfNbues; i++)                                                                    //sachin
    {
      lteHelper->NbAttach (nbueLteDevs.Get (i), enbLteDevs.Get (0));
    }
                                                                                                                  //sachin




  NS_LOG_LOGIC ("setting up applications");

  // Install and start applications on UEs and remote host
  uint16_t dlPort = 10000;
  uint16_t ulPort = 20000;

  // randomize a bit start times to avoid simulation artifacts
  // (e.g., buffer overflows due to packet transmissions happening
  // exactly at the same time)
  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (0));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (0.010));

  for (uint32_t u = 0; u < numberOfUes; ++u)
    {
      Ptr<Node> ue = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      for (uint32_t b = 0; b < numBearersPerUe; ++b)
        {
	  ++dlPort;
          ++ulPort;

          ApplicationContainer clientApps;
          ApplicationContainer serverApps;

          //NS_LOG_LOGIC ("installing UDP DL app for UE " << u);
          //UdpClientHelper dlClientHelper (ueIpIfaces.GetAddress (u), dlPort);
          //clientApps.Add (dlClientHelper.Install (remoteHost));
          //PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
          //                                    InetSocketAddress (Ipv4Address::GetAny (), dlPort));
          //serverApps.Add (dlPacketSinkHelper.Install (ue));

          NS_LOG_LOGIC ("installing UDP UL app for UE " << u);
          UdpClientHelper ulClientHelper (remoteHostAddr, ulPort);
          clientApps.Add (ulClientHelper.Install (ue));
          PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory",
                                               InetSocketAddress (Ipv4Address::GetAny (), ulPort));
          serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

          Ptr<EpcTft> tft = Create<EpcTft> ();
          //EpcTft::PacketFilter dlpf;
          //dlpf.localPortStart = dlPort;
          //dlpf.localPortEnd = dlPort;
          //tft->Add (dlpf);
          EpcTft::PacketFilter ulpf;
          ulpf.remotePortStart = ulPort;
          ulpf.remotePortEnd = ulPort;
          tft->Add (ulpf);
          EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
          lteHelper->ActivateDedicatedEpsBearer (ueLteDevs.Get (u), bearer, tft);

          Time startTime = Seconds (startTimeSeconds->GetValue ());
          serverApps.Start (startTime);
          clientApps.Start (startTime);

        } // end for b
    }



  for (uint32_t u = 0; u > numberOfNbues; ++u)                                                                               //sachin start
    {
      Ptr<Node> nbue = nbueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> nbueStaticRouting = ipv4RoutingHelper.GetStaticRouting (nbue->GetObject<Ipv4> ());
      nbueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      for (uint32_t b = 0; b < numBearersPerUe; ++b)
        {
	  ++dlPort;
          ++ulPort;

          ApplicationContainer clientApps;
          ApplicationContainer serverApps;

          //NS_LOG_LOGIC ("installing UDP DL app for UE " << u);
          //UdpClientHelper dlClientHelper (ueIpIfaces.GetAddress (u), dlPort);
          //clientApps.Add (dlClientHelper.Install (remoteHost));
          //PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
          //                                    InetSocketAddress (Ipv4Address::GetAny (), dlPort));
          //serverApps.Add (dlPacketSinkHelper.Install (ue));

          NS_LOG_LOGIC ("installing UDP UL app for NBUE " << u);
          UdpClientHelper ulClientHelper (remoteHostAddr, ulPort);
          clientApps.Add (ulClientHelper.Install (nbue));
          PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory",
                                               InetSocketAddress (Ipv4Address::GetAny (), ulPort));
          serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

          Ptr<EpcTft> tft = Create<EpcTft> ();
          //EpcTft::PacketFilter dlpf;
          //dlpf.localPortStart = dlPort;
          //dlpf.localPortEnd = dlPort;
          //tft->Add (dlpf);
          EpcTft::PacketFilter ulpf;
          ulpf.remotePortStart = ulPort;
          ulpf.remotePortEnd = ulPort;
          tft->Add (ulpf);
          EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
          lteHelper->ActivateDedicatedEpsBearer (ueLteDevs.Get (u), bearer, tft);

          Time startTime = Seconds (startTimeSeconds->GetValue ());
          serverApps.Start (startTime);
          clientApps.Start (startTime);

        } // end for b
    }                                                                                                         //sachin end




  lteHelper->EnablePhyTraces ();
  lteHelper->EnableMacTraces ();
  lteHelper->EnableRlcTraces ();
  lteHelper->EnablePdcpTraces ();
  Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats ();
  rlcStats->SetAttribute ("EpochDuration", TimeValue (Seconds (2.00)));
  Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
  pdcpStats->SetAttribute ("EpochDuration", TimeValue (Seconds (2.00)));

  // Create the animation object and configure for specified output
 // AnimationInterface anim (animFile);


  Ptr<FlowMonitor> flowMonitor;
  FlowMonitorHelper flowHelper;
  //flowMonitor = flowHelper.InstallAll();
  flowMonitor = flowHelper.Install (ueNodes);
  flowMonitor = flowHelper.Install (remoteHostContainer);
  //Simulator::Schedule(Seconds(i*t_diff), &ThroughputMonitor, &flowHelper, flowMonitor);



   // connect custom trace sinks for RRC connection establishment and handover notification
   Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",                               //sachin
                   MakeCallback (&NotifyConnectionEstablishedEnb));

//  int i;
 // for (i = 1; i <= n_dist; i++) {
//	Simulator::Schedule (Seconds (4), &RelocateUE, ueNodes, pos_diff2);
	//Simulator::Schedule (Seconds (i*t_diff), &ThroughputMonitor, &flowHelper, flowMonitor);
  //}


  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  flowMonitor->SerializeToXmlFile("UeRhUl.xml", true, true);

  Simulator::Destroy ();
  return 0;

}
