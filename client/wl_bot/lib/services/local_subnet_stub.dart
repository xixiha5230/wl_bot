/// Fallback for platforms without dart:io (web): no interface enumeration.
Future<List<String>> localSubnetPrefixes() async => [];
