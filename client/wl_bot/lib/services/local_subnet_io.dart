import 'dart:io';

/// /24 prefixes (e.g. ["192.168.1"]) of the device's own IPv4 interfaces.
Future<List<String>> localSubnetPrefixes() async {
  final prefixes = <String>{};
  final interfaces = await NetworkInterface.list(
    type: InternetAddressType.IPv4,
    includeLoopback: false,
  );
  for (final interface in interfaces) {
    for (final addr in interface.addresses) {
      if (addr.isLoopback || addr.isLinkLocal) {
        continue;
      }
      final parts = addr.address.split('.');
      if (parts.length == 4) {
        prefixes.add('${parts[0]}.${parts[1]}.${parts[2]}');
      }
    }
  }
  return prefixes.toList();
}
