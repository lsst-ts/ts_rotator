.. py:currentmodule:: lsst.ts.rotator

.. _lsst.ts.rotator.developer_guide:

###############
Developer Guide
###############

The Rotator CSC is implemented using `ts_salobj <https://github.com/lsst-ts/ts_salobj>`_ and `ts_hexrotcom <https://ts-hexrotcomm.lsst.io>`_.

.. _lsst.ts.rotator-api:

API
===

The primary class is:

* `RotatorCsc`: the CSC.

.. automodapi:: lsst.ts.rotator
   :no-main-docstr:
   :no-inheritance-diagram:

Build and Test
==============

This is a pure python package. There is nothing to build except the documentation.

.. code-block:: bash

    make_idl_files.py ATDome
    setup -r .
    pytest -v  # to run tests
    package-docs clean; package-docs build  # to build the documentation

Contributing
============

``lsst.ts.rotator`` is developed at https://github.com/lsst-ts/ts_rotator.
You can find Jira issues for this module using `labels=ts_rotator <https://jira.lsstcorp.org/issues/?jql=project%20%3D%20DM%20AND%20labels%20%20%3D%20ts_rotator>`_..
