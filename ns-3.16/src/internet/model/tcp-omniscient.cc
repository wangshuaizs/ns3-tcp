/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 Adrian Sai-wah Tam
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
 * Author: Ruijia Wang based on tcp-newreno
 */

#define NS_LOG_APPEND_CONTEXT \
  if (m_node) { std::clog << Simulator::Now ().GetSeconds () << " [node " << m_node->GetId () << "] "; }

#include "tcp-omniscient.h"
#include "ns3/log.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/simulator.h"
#include "ns3/abort.h"
#include "ns3/node.h"

NS_LOG_COMPONENT_DEFINE ("TcpOmniscient");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (TcpOmniscient);

TypeId
TcpOmniscient::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpOmniscient")
    .SetParent<TcpSocketBase> ()
    .AddConstructor<TcpOmniscient> ()
    .AddAttribute ("ReTxThreshold", "Threshold for fast retransmit",
                    UintegerValue (3),
                    MakeUintegerAccessor (&TcpOmniscient::m_retxThresh),
                    MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("LimitedTransmit", "Enable limited transmit",
            BooleanValue (false),
            MakeBooleanAccessor (&TcpOmniscient::m_limitedTx),
            MakeBooleanChecker ())
    .AddTraceSource ("CongestionWindow",
                     "The TCP connection's congestion window",
                     MakeTraceSourceAccessor (&TcpOmniscient::m_cWnd))
  ;
  return tid;
}

TcpOmniscient::TcpOmniscient (void)
  : m_retxThresh (3), // mute valgrind, actual value set by the attribute system
    m_inFastRec (false),
    m_limitedTx (false) // mute valgrind, actual value set by the attribute system
{
  NS_LOG_FUNCTION (this);
}

TcpOmniscient::TcpOmniscient (const TcpOmniscient& sock)
  : TcpSocketBase (sock),
    m_cWnd (sock.m_cWnd),
    m_ssThresh (sock.m_ssThresh),
    m_initialCWnd (sock.m_initialCWnd),
    m_retxThresh (sock.m_retxThresh),
    m_inFastRec (false),
    m_limitedTx (sock.m_limitedTx)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_LOGIC ("Invoked the copy constructor");
}

TcpOmniscient::~TcpOmniscient (void)
{
}

/** We initialize m_cWnd from this function, after attributes initialized */
int
TcpOmniscient::Listen (void)
{
  NS_LOG_FUNCTION (this);
  InitializeCwnd ();
  return TcpSocketBase::Listen ();
}

/** We initialize m_cWnd from this function, after attributes initialized */
int
TcpOmniscient::Connect (const Address & address)
{
  NS_LOG_FUNCTION (this << address);
  InitializeCwnd ();
  return TcpSocketBase::Connect (address);
}

/** Limit the size of in-flight data by cwnd and receiver's rxwin */
uint32_t
TcpOmniscient::Window (void)
{
  NS_LOG_FUNCTION (this);
  return std::min (m_rWnd.Get (), m_cWnd.Get ());
}

Ptr<TcpSocketBase>
TcpOmniscient::Fork (void)
{
  return CopyObject<TcpOmniscient> (this);
}

/** New ACK (up to seqnum seq) received. Increase cwnd and call TcpSocketBase::NewAck() */
void
TcpOmniscient::NewAck (const SequenceNumber32& seq)
{
  NS_LOG_FUNCTION (this << seq);
  NS_LOG_LOGIC ("TcpOmniscient receieved ACK for seq " << seq <<
                " cwnd " << m_cWnd <<
                " ssthresh " << m_ssThresh);

    if(TraceMe) std::cout << Simulator::Now().GetSeconds() << " " << GetNode()->GetId() <<  " SINGLETCP CWND " << m_cWnd << " " << Window() <<  std::endl;

  // Check for exit condition of fast recovery
  if (m_inFastRec && seq < m_recover)
    { // Partial ACK, partial window deflation (RFC2582 sec.3 bullet #5 paragraph 3)
      m_cWnd -= seq - m_txBuffer.HeadSequence ();
      m_cWnd += m_segmentSize;  // increase cwnd
      NS_LOG_INFO ("Partial ACK in fast recovery: cwnd set to " << m_cWnd);
      TcpSocketBase::NewAck (seq); // update m_nextTxSequence and send new data if allowed by window
      DoRetransmit (); // Assume the next seq is lost. Retransmit lost packet
      return;
    }
  else if (m_inFastRec && seq >= m_recover)
    { // Full ACK (RFC2582 sec.3 bullet #5 paragraph 2, option 1)
      m_cWnd = std::min (m_ssThresh, BytesInFlight () + m_segmentSize);
      m_inFastRec = false;
      NS_LOG_INFO ("Received full ACK. Leaving fast recovery with cwnd set to " << m_cWnd);
    }

  // Increase of cwnd based on current phase (slow start or congestion avoidance)
  if (m_cWnd < m_ssThresh)
    { // Slow start mode, add one segSize to cWnd. Default m_ssThresh is 65535. (RFC2001, sec.1)
      m_cWnd += m_segmentSize;
      NS_LOG_INFO ("In SlowStart, updated to cwnd " << m_cWnd << " ssthresh " << m_ssThresh);
    }
  else
    { // Congestion avoidance mode, increase by (segSize*segSize)/cwnd. (RFC2581, sec.3.1)
      // To increase cwnd for one segSize per RTT, it should be (ackBytes*segSize)/cwnd
      double adder = static_cast<double> (m_segmentSize * m_segmentSize) / m_cWnd.Get ();
      adder = std::max (1.0, adder);
      m_cWnd += static_cast<uint32_t> (adder);
      NS_LOG_INFO ("In CongAvoid, updated to cwnd " << m_cWnd << " ssthresh " << m_ssThresh);
    }

  // Complete newAck processing
  TcpSocketBase::NewAck (seq);
}

/** Cut cwnd and enter fast recovery mode upon triple dupack */
void
TcpOmniscient::DupAck (const TcpHeader& t, uint32_t count)
{
  NS_LOG_FUNCTION (this << count);
  if (count == m_retxThresh && !m_inFastRec)
    { // triple duplicate ack triggers fast retransmit (RFC2582 sec.3 bullet #1)
      m_ssThresh = std::max (2 * m_segmentSize, BytesInFlight () / 2);
      m_cWnd = m_ssThresh + 3 * m_segmentSize;
      m_recover = m_highTxMark;
      m_inFastRec = true;
      NS_LOG_INFO ("Triple dupack. Enter fast recovery mode. Reset cwnd to " << m_cWnd <<
                   ", ssthresh to " << m_ssThresh << " at fast recovery seqnum " << m_recover);
      DoRetransmit ();
    }
  else if (m_inFastRec)
    { // Increase cwnd for every additional dupack (RFC2582, sec.3 bullet #3)
      m_cWnd += m_segmentSize;
      NS_LOG_INFO ("Dupack in fast recovery mode. Increase cwnd to " << m_cWnd);
      SendPendingData (m_connected);
    }
  else if (!m_inFastRec && m_limitedTx && m_txBuffer.SizeFromSequence (m_nextTxSequence) > 0)
    { // RFC3042 Limited transmit: Send a new packet for each duplicated ACK before fast retransmit
      NS_LOG_INFO ("Limited transmit");
      uint32_t sz = SendDataPacket (m_nextTxSequence, m_segmentSize, true);
      m_nextTxSequence += sz;                    // Advance next tx sequence
    };
}

/** Retransmit timeout */
void
TcpOmniscient::Retransmit (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_LOGIC (this << " ReTxTimeout Expired at time " << Simulator::Now ().GetSeconds ());
  m_inFastRec = false;

  // If erroneous timeout in closed/timed-wait state, just return
  if (m_state == CLOSED || m_state == TIME_WAIT) return;
  // If all data are received (non-closing socket and nothing to send), just return
  if (m_state <= ESTABLISHED && m_txBuffer.HeadSequence () >= m_highTxMark) return;

  // According to RFC2581 sec.3.1, upon RTO, ssthresh is set to half of flight
  // size and cwnd is set to 1*MSS, then the lost packet is retransmitted and
  // TCP back to slow start
  m_ssThresh = std::max (2 * m_segmentSize, BytesInFlight () / 2);
  m_cWnd = m_segmentSize;
  m_nextTxSequence = m_txBuffer.HeadSequence (); // Restart from highest Ack
  NS_LOG_INFO ("RTO. Reset cwnd to " << m_cWnd <<
               ", ssthresh to " << m_ssThresh << ", restart from seqnum " << m_nextTxSequence);
  m_rtt->IncreaseMultiplier ();             // Double the next RTO
  DoRetransmit ();                          // Retransmit the packet
}

void
TcpOmniscient::SetSegSize (uint32_t size)
{
  NS_ABORT_MSG_UNLESS (m_state == CLOSED, "TcpOmniscient::SetSegSize() cannot change segment size after connection started.");
  m_segmentSize = size;
}

void
TcpOmniscient::SetSSThresh (uint32_t threshold)
{
  m_ssThresh = threshold;
}

uint32_t
TcpOmniscient::GetSSThresh (void) const
{
  return m_ssThresh;
}

void
TcpOmniscient::SetInitialCwnd (uint32_t cwnd)
{
  NS_ABORT_MSG_UNLESS (m_state == CLOSED, "TcpOmniscient::SetInitialCwnd() cannot change initial cwnd after connection started.");
  m_initialCWnd = cwnd;
}

uint32_t
TcpOmniscient::GetInitialCwnd (void) const
{
  return m_initialCWnd;
}

void
TcpOmniscient::InitializeCwnd (void)
{
  /*
   * Initialize congestion window, default to 1 MSS (RFC2001, sec.1) and must
   * not be larger than 2 MSS (RFC2581, sec.3.1). Both m_initiaCWnd and
   * m_segmentSize are set by the attribute system in ns3::TcpSocket.
   */
  m_cWnd = m_initialCWnd * m_segmentSize;
}

//compute the link
int
Find_Link(int node1, int node2, int n1[], int n2[]){
  int i;
  for(i = 0; (node1 != n1[i] || node2 != n2[i]) && (node1 != n2[i] && node2 != n1[i]); i++){

  }
  //cout<<"start find link ="<<i<<endl;
  return i;

}

//Find the available BW
uint32_t
Find_BW (int sender, int dest)
{
  ifstream rt_in, tp_in, eh_in, lu_in;
  int routingtable[10][10][10] = {0};
  int s, d, NODES, LINKS;
  //This file is calculated by the SPF algorithm based on the known topology
  rt_in.open("routingtable111.txt");
  if(!rt_in.is_open()) {
        cout<<"error opening rt"<<endl;
        return 0;
  }


  //reading the routingtable
  for(int i = 0, num = 0; i < 90; i++){
    //fscanf(fp, "%d", &s);
    //fscanf(fp, "%d", &d);
    //fscanf(fp, "%d", &num);

    rt_in>>s>>d>>num;
    //cout<<"s = "<<s<<"d = "<<d<<"num = "<<num<<endl;
    for(int j = 0; j < num; j++){
      //fscanf(fp, "%d", &routingtable[s-1][d-1][j]);
      rt_in>>routingtable[s-1][d-1][j];
      //cout<<"node:"<<routingtable[s-1][d-1][j]<<endl;
    }
  }


  //reading topology
  tp_in.open("internet2-fanout10/internet2-withbandwidthdelay-fanout10.txt");
  if(!tp_in.is_open())
  {
        cout<<"error opening tp"<<endl;
        return 0;
  }

  //fscanf(fp1, "%d", &NODES);
  tp_in>>NODES;
  //cout<<"NODES = "<<NODES<<endl;
  //fscanf(fp1, "%d", &LINKS);
  tp_in>>LINKS;
  //cout<<"LINKS = "<<LINKS<<endl;

  int linkDelays[LINKS];
  int linkBandwidths[LINKS];

  int n1[LINKS], n2[LINKS];
  for(int i=0; i<LINKS; i++)
  {
    //fscanf(fp1, "%d", &n1[i]);
    //fscanf(fp1, "%d", &n2[i]);
    //fscanf(fp1, "%d", &linkBandwidths[i]);
    //fscanf(fp1, "%d", &linkDelays[i]);
    tp_in>>n1[i]>>n2[i]>>linkBandwidths[i]>>linkDelays[i];
    //cout<<n1[i]<<"\t"<<n2[i]<<"\t"<<linkBandwidths[i]<<"\t"<<linkDelays[i]<<endl;

  }

  //read endhost
  eh_in.open("internet2-fanout10/internet2-endhosts.txt");
  if(!eh_in.is_open()) {
        cout<<"error opening eh"<<endl;
        return 0;
  }
  int host_interfaces[NODES];
  //int host_interfaceIdx[NODES];
  int numhosts;
  int host, link, idx;
  //fscanf(fp2, "%d", &numhosts);
  eh_in>>numhosts;
  for(int i=0; i<numhosts; i++)
  {
    //fscanf(fp2, "%d %d %d", &host, &link, &idx);
    eh_in>>host>>link>>idx;
    host_interfaces[host] = link;
    //host_interfaceIdx[host] = idx;
  }

  //int sender, dest;
  int link_s, link_d;
  int node_s, node_d;
  link_s = host_interfaces[sender];
  link_d = host_interfaces[dest];
  node_s = n1[link_s];
  node_d = n1[link_d];

  lu_in.open("Link_Util.txt");
  if(!rt_in.is_open()) {
        cout<<"error opening rt"<<endl;
        return 0;
  }
  //read in the link utilisation
  double time[LINKS];
  int linkid[LINKS], linkutil[LINKS];
  for(int i = 0; i < LINKS; i++){
    //fscanf(fp3, "%lf\t%d\t%d", &time[i], &linkid[i], &linkutil[i]);
    lu_in>>time[i]>>linkid[i]>>linkutil[i];
  }

  int temp0 = routingtable[node_s][node_d][0];
  int temp1 = routingtable[node_s][node_d][1];
  int temp  = linkutil[Find_Link(temp0, temp1, n1, n2)];
  for(int i = 1; routingtable[node_s][node_d][i] != 0; i++){
    temp0 = routingtable[node_s][node_d][i-1];
    temp1 = routingtable[node_s][node_d][i];
    if(temp > linkutil[Find_Link(temp0, temp1, n1, n2)])
      temp = linkutil[Find_Link(temp0, temp1, n1, n2)];
  }
  //cout<<"find bw"<<endl;
  rt_in.close();
  tp_in.close();
  eh_in.close();
  lu_in.close();
  return temp;
}

//Calculate the node number
int CalcNodeNum(uint32_t address){
  int num;
  ifstream in;
  num = (address - 167772162) / 256 ;
  //cout<<"node number = "<<num<<endl;
  //read endhost
  in.open("internet2-fanout10/internet2-endhosts.txt");
  if(!in.is_open()) {
        cout<<"error opening calc"<<endl;
        return 0;
  }

  int host_interfaces[110];
  //int host_interfaceIdx[NODES];
  int numhosts;
  int host, link, idx;
  //fscanf(fpc, "%d", &numhosts);
  in>>numhosts;
  for(int i=0; i<numhosts; i++)
  {
    //fscanf(fpc, "%d %d %d", &host, &link, &idx);
    in>>host>>link>>idx;
    host_interfaces[host] = link;
    //host_interfaceIdx[host] = idx;
  }
  int n;
  for(n = 10; host_interfaces[n] != num; n++)
  {
  }
  //cout<<"n = "<<n<<endl;
  in.close();
  return n;
}

/** Received a packet upon SYN_SENT */
void
TcpSocketBase::ProcessSynSent (Ptr<Packet> packet, const TcpHeader& tcpHeader)
{
  NS_LOG_FUNCTION (this << tcpHeader);
  uint32_t mycwnd;

  // Extract the flags. PSH and URG are not honoured.
  uint8_t tcpflags = tcpHeader.GetFlags () & ~(TcpHeader::PSH | TcpHeader::URG);

  if (tcpflags == 0)
    { // Bare data, accept it and move to ESTABLISHED state. This is not a normal behaviour. Remove this?
      NS_LOG_INFO ("SYN_SENT -> ESTABLISHED");
      m_state = ESTABLISHED;
      m_connected = true;
      m_retxEvent.Cancel ();
      m_delAckCount = m_delAckMaxCount;
      ReceivedData (packet, tcpHeader);
      Simulator::ScheduleNow (&TcpSocketBase::ConnectionSucceeded, this);
    }
  else if (tcpflags == TcpHeader::ACK)
    { // Ignore ACK in SYN_SENT
    }
  else if (tcpflags == TcpHeader::SYN)
    { // Received SYN, move to SYN_RCVD state and respond with SYN+ACK
      NS_LOG_INFO ("SYN_SENT -> SYN_RCVD");
      m_state = SYN_RCVD;
      m_cnCount = m_cnRetries;
      m_rxBuffer.SetNextRxSequence (tcpHeader.GetSequenceNumber () + SequenceNumber32 (1));
      SendEmptyPacket (TcpHeader::SYN | TcpHeader::ACK);
    }
  else if (tcpflags == (TcpHeader::SYN | TcpHeader::ACK)
           && m_nextTxSequence + SequenceNumber32 (1) == tcpHeader.GetAckNumber ())
    { // Handshake completed
      NS_LOG_INFO ("SYN_SENT -> ESTABLISHED");
      m_state = ESTABLISHED;
      mycwnd = Find_BW(CalcNodeNum(m_endPoint->GetLocalAddress().Get()), CalcNodeNum(m_endPoint->GetPeerAddress().Get()));
      if((double)mycwnd * (double)m_segmentSize > m_cWnd)
        m_cWnd = (uint32_t)((double)(mycwnd) * (double)m_segmentSize);
      m_connected = true;
      m_retxEvent.Cancel ();
      m_rxBuffer.SetNextRxSequence (tcpHeader.GetSequenceNumber () + SequenceNumber32 (1));
      m_highTxMark = ++m_nextTxSequence;
      m_txBuffer.SetHeadSequence (m_nextTxSequence);
      SendEmptyPacket (TcpHeader::ACK);
      SendPendingData (m_connected);
      Simulator::ScheduleNow (&TcpSocketBase::ConnectionSucceeded, this);
      // Always respond to first data packet to speed up the connection.
      // Remove to get the behaviour of old NS-3 code.
      m_delAckCount = m_delAckMaxCount;
    }
  else
    { // Other in-sequence input
      if (tcpflags != TcpHeader::RST)
        { // When (1) rx of FIN+ACK; (2) rx of FIN; (3) rx of bad flags
          NS_LOG_LOGIC ("Illegal flag " << std::hex << static_cast<uint32_t> (tcpflags) << std::dec << " received. Reset packet is sent.");
          SendRST ();
        }
      CloseAndNotify ();
    }
}

/** Received a packet upon SYN_RCVD */
void
TcpSocketBase::ProcessSynRcvd (Ptr<Packet> packet, const TcpHeader& tcpHeader,
                               const Address& fromAddress, const Address& toAddress)
{
  NS_LOG_FUNCTION (this << tcpHeader);
  uint32_t mycwnd;

  // Extract the flags. PSH and URG are not honoured.
  uint8_t tcpflags = tcpHeader.GetFlags () & ~(TcpHeader::PSH | TcpHeader::URG);

  if (tcpflags == 0
      || (tcpflags == TcpHeader::ACK
          && m_nextTxSequence + SequenceNumber32 (1) == tcpHeader.GetAckNumber ()))
    { // If it is bare data, accept it and move to ESTABLISHED state. This is
      // possibly due to ACK lost in 3WHS. If in-sequence ACK is received, the
      // handshake is completed nicely.
      NS_LOG_INFO ("SYN_RCVD -> ESTABLISHED");
      m_state = ESTABLISHED;
      mycwnd = Find_BW(CalcNodeNum(m_endPoint->GetLocalAddress().Get()), CalcNodeNum(m_endPoint->GetPeerAddress().Get()));
      if((double)mycwnd * (double)m_segmentSize > m_cWnd)
        m_cWnd = (uint32_t)((double)(mycwnd) * (double)m_segmentSize);
      m_connected = true;
      m_retxEvent.Cancel ();
      m_highTxMark = ++m_nextTxSequence;
      m_txBuffer.SetHeadSequence (m_nextTxSequence);
      if (m_endPoint)
        {
          m_endPoint->SetPeer (InetSocketAddress::ConvertFrom (fromAddress).GetIpv4 (),
                               InetSocketAddress::ConvertFrom (fromAddress).GetPort ());
        }
      else if (m_endPoint6)
        {
          m_endPoint6->SetPeer (Inet6SocketAddress::ConvertFrom (fromAddress).GetIpv6 (),
                                Inet6SocketAddress::ConvertFrom (fromAddress).GetPort ());
        }
      // Always respond to first data packet to speed up the connection.
      // Remove to get the behaviour of old NS-3 code.
      m_delAckCount = m_delAckMaxCount;
      ReceivedAck (packet, tcpHeader);
      NotifyNewConnectionCreated (this, fromAddress);
      // As this connection is established, the socket is available to send data now
      if (GetTxAvailable () > 0)
        {
          NotifySend (GetTxAvailable ());
        }
    }
  else if (tcpflags == TcpHeader::SYN)
    { // Probably the peer lost my SYN+ACK
      m_rxBuffer.SetNextRxSequence (tcpHeader.GetSequenceNumber () + SequenceNumber32 (1));
      SendEmptyPacket (TcpHeader::SYN | TcpHeader::ACK);
    }
  else if (tcpflags == (TcpHeader::FIN | TcpHeader::ACK))
    {
      if (tcpHeader.GetSequenceNumber () == m_rxBuffer.NextRxSequence ())
        { // In-sequence FIN before connection complete. Set up connection and close.
          m_connected = true;
          m_retxEvent.Cancel ();
          m_highTxMark = ++m_nextTxSequence;
          m_txBuffer.SetHeadSequence (m_nextTxSequence);
          if (m_endPoint)
            {
              m_endPoint->SetPeer (InetSocketAddress::ConvertFrom (fromAddress).GetIpv4 (),
                                   InetSocketAddress::ConvertFrom (fromAddress).GetPort ());
            }
          else if (m_endPoint6)
            {
              m_endPoint6->SetPeer (Inet6SocketAddress::ConvertFrom (fromAddress).GetIpv6 (),
                                    Inet6SocketAddress::ConvertFrom (fromAddress).GetPort ());
            }
          PeerClose (packet, tcpHeader);
        }
    }
  else
    { // Other in-sequence input
      if (tcpflags != TcpHeader::RST)
        { // When (1) rx of SYN+ACK; (2) rx of FIN; (3) rx of bad flags
          NS_LOG_LOGIC ("Illegal flag " << tcpflags << " received. Reset packet is sent.");
          if (m_endPoint)
            {
              m_endPoint->SetPeer (InetSocketAddress::ConvertFrom (fromAddress).GetIpv4 (),
                                   InetSocketAddress::ConvertFrom (fromAddress).GetPort ());
            }
          else if (m_endPoint6)
            {
              m_endPoint6->SetPeer (Inet6SocketAddress::ConvertFrom (fromAddress).GetIpv6 (),
                                    Inet6SocketAddress::ConvertFrom (fromAddress).GetPort ());
            }
          SendRST ();
        }
      CloseAndNotify ();
    }
}

} // namespace ns3

