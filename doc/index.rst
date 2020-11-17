.. py:currentmodule:: lsst.ts.mtrotator

.. _lsst.ts.mtrotator:

#################
lsst.ts.mtrotator
#################

.. image:: https://img.shields.io/badge/Project Metadata-gray.svg
    :target: https://ts-xml.lsst.io/index.html#index-master-csc-table-mtrotator
.. image:: https://img.shields.io/badge/SAL\ Interface-gray.svg
    :target: https://ts-xml.lsst.io/sal_interfaces/MTRotator.html
.. image:: https://img.shields.io/badge/GitHub-gray.svg
    :target: https://github.com/lsst-ts/ts_mtrotator
.. image:: https://img.shields.io/badge/Jira-gray.svg
    :target: https://jira.lsstcorp.org/issues/?jql=labels+%3D+ts_mtrotator

Overview
========

The MTRotator CSC controls the camera rotator on the Simonyi Survey Telescope.

User Guide
==========

Start the MTRotator CSC as follows:

.. prompt:: bash

    run_mtrotator.py

Then check that the CSC has control of the low-level controller, as follows:

* Wait for the ``connected`` event to report ``command=True`` and ``telemetry=True``.
  This should happen quickly; if it does not then check that the low-level controller is fully booted up and configured to use the correct IP address for the CSC.
* Check the ``controllerState`` event.
  If it is ``state=Offline, offline_substate=PublishOnly``, which is the state the low-level controller wakes up in,
  then you must :ref:`use the EUI <lsst.ts.mtrotator.eui>` to change the state.
* Check the ``commandableByDDS`` event.
  If ``state=False`` then you must :ref:`use the EUI <lsst.ts.mtrotator.eui>` to change the control mode.

Notes
-----

* To recover from the ``FAULT`` state (after fixing whatever is wrong) issue the ``clearError`` command.
  This will transition to the ``STANDBY`` state.

* The low-level controller maintains the CSC summary state,
  so the CSC reports a summary state of ``OFFLINE`` until it receives telemetry from the low-level controller.
  Thus the CSC may transition from ``OFFLINE`` to almost any other state as it starts up.

* Communication between the low-level controller and CSC is quite unusual:

  * The low-level controller connects to a TCP/IP *server* in the CSC.
    Thus the low-level controller must be configured with the TCP/IP address of the CSC.
  * The low-level controller does not acknowledge commands in any way.
    Thus the CSC must try to predict whether the low-level controller can execute a command and reject the command if not.
    Unfortunately this prediction cannot be completely accurate.
  * The connection uses two separate sockets, one for commands and the other for telemetry and configuration.
    Both are one-directional: the low-level controller reads commands on the command socket and writes configuration and telemetry to the telemetry socket.

Simulator
---------

The CSC includes a simulation mode. To run using CSC's internal simulator:

.. prompt:: bash

    run_mtrotator.py --simulate

.. _lsst.ts.mtrotator.eui:

The Engineering User Interface (EUI)
------------------------------------

Use the Engineering User Interface (EUI) as follows to enable CSC control of the low-level controller:

    * State must be ``state=Offline, offline_substate=Available`` or any more enabled state.
      The low-level controller wakes up in ``state=Offline, offline_substate=PublishOnly``,
      and you must change this before the CSC can control the low-level controller.
      Change the state on the main panel of the EUI.
    * Control mode must be ``DDS``.
      The low-level controller wakes up in control mode ``GUI``,
      and you must change this before the CSC can control the low-level controller.
      To change the control mode use the ``Parameters`` panel;
      note that the EUI *shows* the control mode on the main panel, but that display is read-only.

Developer Guide
===============

.. toctree::
    developer_guide
    :maxdepth: 1

Version History
===============

.. toctree::
    version_history
    :maxdepth: 1
