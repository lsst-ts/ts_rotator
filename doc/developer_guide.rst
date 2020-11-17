.. py:currentmodule:: lsst.ts.mtrotator

.. _lsst.ts.mtrotator.developer_guide:

###############
Developer Guide
###############

The MTRotator CSC is implemented using `ts_salobj <https://github.com/lsst-ts/ts_salobj>`_ and `ts_hexrotcom <https://ts-hexrotcomm.lsst.io>`_.

.. _lsst.ts.mtrotator-api:

API
===

The primary class is:

* `RotatorCsc`: the CSC.

.. automodapi:: lsst.ts.mtrotator
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

``lsst.ts.mtrotator`` is developed at https://github.com/lsst-ts/ts_mtrotator.
You can find Jira issues for this module using `labels=ts_mtrotator <https://jira.lsstcorp.org/issues/?jql=project%20%3D%20DM%20AND%20labels%20%20%3D%20ts_mtrotator>`_..
