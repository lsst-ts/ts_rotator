.. py:currentmodule:: lsst.ts.rotator

.. _lsst.ts.rotator:

###############
lsst.ts.rotator
###############

Python Commandable SAL Component (CSC) for the LSST main telescope rotator.
This code is a front end for a low level controller written by Moog.

To use the CSC:

* The low level controller wakes up in the state OFFLINE/PUBLISH_ONLY and immediately tries to connect to the CSC (two separate connections, one for commands and one for telemetry and configuration).
  The state of the connection is given by the ``connected`` event.
* In this OFFLINE/PUBLISH_ONLY state the low level controller will not accept any comments from the CSC.
  It will merely publish telemetry and configuration.
* It is necessary to use a LabVIEW engineering user interface (EUI) to transition from OFFLINE/PUBLISH_ONLY to OFFLINE/AVAILABLE before the CSC can control the low-level controller.
* If the low level controller goes into a FAULT state send, recovery requires using the EUI.
  You can either use the EUI for full recovery,
  or send the CSC command ``clearError`` to transition to OFFLINE/PUBLISH_ONLY, then use the EUI to transition to OFFLINE/AVAILABLE.

Here are a few internal details that may help you better understand the system:

* Communication between the low level controller and CSC is quite unusual:

  * The low level controller connects to a TCP/IP _server_ in the CSC.
  * The connection uses two separate sockets, one for commands and the other for telemetry and configuration.
  * The low level controller does not acknowledge commands in any way
    (it only reads from the command socket, it does not write anything to it).
    If the CSC predicts that the low level controller will reject (ignore) a command, it will fail the command (instead of sending it to the controller).
    But this prediction cannot be completely accurate.

* The low level controller maintains the CSC summary state.
  The CSC reports a summary state of OFFLINE until it receives telemetry from the low level controller.
  Thus the CSC may unexpectedly transition from OFFLINE to a different state as it starts up.

.. .. _lsst.ts.rotator-using:

.. Using lsst.ts.rotator
.. =====================

.. toctree linking to topics related to using the module's APIs.

.. .. toctree::
..    :maxdepth: 1

.. _lsst.ts.rotator-contributing:

Contributing
============

``lsst.ts.rotator`` is developed at https://github.com/lsst-ts/ts_rotator.
You can find Jira issues for this module at `labels=ts_salobj <https://jira.lsstcorp.org/issues/?jql=project%20%3D%20DM%20AND%20labels%20%20%3D%20ts_rotator>`_.

.. .. _lsst.ts.rotator-pyapi:

Python API reference
====================

.. automodapi:: lsst.ts.rotator
   :no-main-docstr:
   :no-inheritance-diagram:
