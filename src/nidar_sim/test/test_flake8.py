# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    # SCOPE: this package's suite lints THIS package. GUI/ and
    # humandetection/ are separate ROS packages that live in the same repo and
    # ship their own test/ directories - linting them from here means
    # nidar_sim's suite goes red for defects in code it does not own, and the
    # team stops trusting the signal. Frontier_Exploration_Sim/ is kept as a
    # reference copy of the pre-package scripts, and sim/ is a committed build
    # tree, so neither is linted either.
    excludes = [
        'GUI', 'humandetection', 'Frontier_Exploration_Sim',
        'sim', 'build', 'install', 'log', 'images', 'patches',
    ]

    # max-line-length is raised to 120 (ament's default is 99) for the reason
    # documented in setup.cfg. Everything else is ament's default rule set.
    rc, errors = main_with_errors(
        argv=['--linelength', '120', '--exclude'] + excludes)
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
