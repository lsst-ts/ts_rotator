# This file is part of ts_mtrotator.
#
# Developed for the LSST Telescope and Site Systems.
# This product includes software developed by the LSST Project
# (https://www.lsst.org).
# See the COPYRIGHT file at the top-level directory of this distribution
# for details of code ownership.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import unittest
import pathlib

import jsonschema
import yaml

from lsst.ts import salobj


class ValidationTestCase(unittest.TestCase):
    """Test validation of the config schema."""

    def setUp(self):
        schemaname = "MTRotator.yaml"
        pkg_dir = pathlib.Path(__file__).parents[1]
        schemapath = pkg_dir / "schema" / schemaname
        with open(schemapath, "r") as f:
            rawschema = f.read()
        self.schema = yaml.safe_load(rawschema)
        self.validator = salobj.DefaultingValidator(schema=self.schema)
        self.default = dict(max_ccw_following_error=2.2, num_ccw_following_errors=3,)

    def test_default(self):
        result = self.validator.validate(None)
        for field, expected_value in self.default.items():
            self.assertEqual(result[field], expected_value)

    def test_some_specified(self):
        data = dict(max_ccw_following_error=1.5, num_ccw_following_errors=1,)
        for field, value in data.items():
            one_field_data = {field: value}
            with self.subTest(one_field_data=one_field_data):
                result = self.validator.validate(one_field_data)
                for field, default_value in self.default.items():
                    if field in one_field_data:
                        self.assertEqual(result[field], one_field_data[field])
                    else:
                        self.assertEqual(result[field], default_value)

    def test_all_specified(self):
        data = dict(max_ccw_following_error=1.5, num_ccw_following_errors=1,)
        data_copy = data.copy()
        result = self.validator.validate(data)
        self.assertEqual(data, data_copy)
        for field, value in data.items():
            self.assertEqual(result[field], value)

    def test_invalid_configs(self):
        for name, badval in (
            ("max_ccw_following_error", "oops"),  # Wrong type
            ("max_ccw_following_error", 0),  # Not positive
            ("num_ccw_following_error", "oops"),  # Wrong type
            ("num_ccw_following_error", 0),  # Not positive
        ):
            bad_data = {name: badval}
            with self.subTest(bad_data=bad_data):
                with self.assertRaises(jsonschema.exceptions.ValidationError):
                    self.validator.validate(bad_data)


if __name__ == "__main__":
    unittest.main()
