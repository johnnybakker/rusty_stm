use smoltcp::{
	wire::{HardwareAddress, IpAddress},
	wire::{IpCidr, Ipv4Address, Ipv4Cidr}, 
	iface::{Interface, SocketStorage}
};
use super::hal::ethernet::EthernetDMA;


pub struct Net<'a> {
	pub neighbor_cache: [Option<(IpAddress, smoltcp::iface::Neighbor)>; 16],
	pub routes_storage: [Option<(IpCidr, smoltcp::iface::Route)>; 16],
	pub sockets: [SocketStorage<'a>; 1],
	pub ip_addrs: [IpCidr; 1]
}

impl<'a> Net<'a> {
    pub const fn new() -> Self {
        Self { 
			neighbor_cache: [None; 16], 
			routes_storage: [None; 16], 
			sockets: [SocketStorage::EMPTY], 
			ip_addrs: [IpCidr::Ipv4(Ipv4Cidr::new(Ipv4Address::BROADCAST, 0))] 
		}
    }

	pub fn create_interface(&'a mut self, dma: EthernetDMA<'a, 4, 4>, mac_address: HardwareAddress, ip: &[u8]) -> Interface<'a, EthernetDMA<'a, 4, 4>> {
		self.ip_addrs = [IpCidr::new(Ipv4Address::from_bytes(&ip).into(), 0)];

		let neighbor_cache = smoltcp::iface::NeighborCache::new(&mut self.neighbor_cache[..]);
        let routes = smoltcp::iface::Routes::new(&mut self.routes_storage[..]);
        
		smoltcp::iface::InterfaceBuilder::new(dma, &mut self.sockets[..])
			.hardware_addr(mac_address.into())
			.ip_addrs(&mut self.ip_addrs[..])
			.neighbor_cache(neighbor_cache)
			.routes(routes)
			.finalize()
	}
}