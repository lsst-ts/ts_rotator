.. py:currentmodule:: lsst.ts.rotator

.. _lsst.ts.rotator:

###############
lsst.ts.rotator
###############

Python Commandable SAL Component (CSC) for the LSST main telescope rotator.
This code is a front end for a low level controller written by Moog.

How to start the system:

* Turn on the rotator PXI if it is not already running.
* Start the rotator CSC.
* Wait for the ``connected`` event to report ``command=True`` and ``telemetry=True``.
  This should happen quickly; if it does not then check that the PXI is fully booted up and configured to use the correct IP address for the CSC.
* Check the ``controllerState`` event.
  If it is ``state=Offline, offline_substate=PublishOnly``, which is the state the PXI wakes up in, you must use the vendor's engineering user interface (EUI) to change the state to ``state=Offline, offline_substate=Available`` (or any more enabled mode).
  You can set the state on the main panel.
* Check the ``commandableByDDS`` event.
  If ``state=False`` then you must use the EUI to change the control mode from ``GUI`` to ``DDS``.
  Use the ``Parameters`` panel to change the control mode (though the EUI _shows_ the control mode on the main panel).

Other notes:

* Recovery from the ``FAULT`` state is not standard: to leave the FAULT state send you may send the ``clearError`` command to transition ``state=Offline, offline_substate=PublishOnly``, or use the EUI to recover. Either way you will have to use the EUI to transition to ``state=Offline, offline_substate=Available`` to enable CSC has control.

* The low level controller maintains the CSC summary state.
  The CSC reports a summary state of OFFLINE until it receives telemetry from the low level controller.
  Thus the CSC may unexpectedly transition from OFFLINE to a different state as it starts up.

* Communication between the low level controller and CSC is quite unusual:

  * The low level controller connects to a TCP/IP _server_ in the CSC.
  * The connection uses two separate sockets, one for commands and the other for telemetry and configuration.
  * The low level controller does not acknowledge commands in any way
    (it only reads from the command socket, it does not write anything to it).
    If the CSC predicts that the low level controller will reject (ignore) a command, it will fail the command (instead of sending it to the controller).
    But this prediction cannot be completely accurate.

.. .. toctree::
..    :maxdepth: 1

.. _lsst.ts.rotator-contributing:

Contributing
============

``lsst.ts.rotator`` is developed at https://github.com/lsst-ts/ts_rotator.
You can find Jira issues for this module at `labels=ts_rotator <https://jira.lsstcorp.org/issues/?jql=project%20%3D%20DM%20AND%20labels%20%20%3D%20ts_rotator>`_.

.. _lsst.ts.rotator-pyapi:

Python API reference
====================

.. automodapi:: lsst.ts.rotator
   :no-main-docstr:
   :no-inheritance-diagram:
