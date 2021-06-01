# BTMesh_VendorSpecificGroupBroadcaster

NCP Proxy device meant to be both publisher and broadcaster to the same group
Its model configuration is done using [this provisionner](https://github.com/brian-silabs/BTMesh_HostADVProvisioner)

It starts sending unprovisionned mesh beacons
After successful provisioning or init (once provisioned), it periodically sends custom data to other devices in the group

## Disclaimer ##

The Gecko SDK suite supports development with Silicon Labs IoT SoC and module devices. Unless otherwise specified in the specific directory, all examples are considered to be EXPERIMENTAL QUALITY which implies that the code provided in the repos has not been formally tested and is provided as-is.  It is not suitable for production environments.  In addition, this code will not be maintained and there may be no bug maintenance planned for these resources. Silicon Labs may update projects from time to time.