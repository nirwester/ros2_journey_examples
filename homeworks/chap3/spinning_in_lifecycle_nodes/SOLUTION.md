Which risk do you face if you manually call "rclcpp::spin_once()"
in the constructor of a lifecycle-managed node?

Answer:

Lifecycle managed nodes are usually ran in parallel with a "lifecycle
manager" that controls their initialization. It is common for the
manager to automatically request the configuration of the managed
nodes at start-up.

When creating a lifecycle managed node inheriting from the
corresponding base class, the service-servers required for the
communication with the manager (the ones bounded to the on_configure(),
on_activate()... callbacks) are started before the code in the
constructor of the inheriting class (your node) is executed.

The manager can potentially start and requests the configuration of
the nodes it's managing before these have actually completed the
execution of their constructor.

If for any reason you call spin_once() during the construction of your
node, this could consequently result in the node's configuration
callback to be executed, even if the construction of the node is
not yet completed.

You probably don't want to configure your class before it's fully
constructed: this could result in any sort of issue, including
black-holes and portals bringing you to another dimension where
Nutella doesn't exist.

So as a rule of thumb: don't call any spin function in the constructor
of a managed node!