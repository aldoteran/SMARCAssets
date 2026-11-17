using System.Text;
using SensorProtocols.Transport;

public class DebugUdpPublisher : UdpPublisher
{
    protected override byte[] CreatePacket(double simulationTime)
    {
        return Encoding.ASCII.GetBytes("Bajskorv!!");
    }
}
