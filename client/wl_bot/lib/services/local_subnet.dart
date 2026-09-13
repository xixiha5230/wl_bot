import 'local_subnet_stub.dart'
    if (dart.library.io) 'local_subnet_io.dart' as impl;

/// /24 prefixes (e.g. ["192.168.1"]) of the device's own IPv4 interfaces.
/// Empty on platforms where interface enumeration is unavailable (web).
Future<List<String>> localSubnetPrefixes() => impl.localSubnetPrefixes();
