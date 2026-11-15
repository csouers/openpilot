"""The jetlink seam: jetlink's own checks of the fork's adapter module.

Jetlink publishes what it holds a fork to (jetlink/openpilot/testing.py); these run
the part that applies to this fork. The link is off in a test, so every hook must
answer inertly and nothing may reach the device's params store.
"""
import pytest

from jetlink.openpilot.testing import check_adapter, check_fallback, check_seam, install_guard

from openpilot import jetlink_adapter

GUARD = install_guard()


@pytest.fixture(autouse=True)
def no_machine_files(tmp_path, monkeypatch):
  # link_on() reads the setting as a file: keep it off this machine's store
  monkeypatch.setenv('PARAMS_ROOT', str(tmp_path / 'params'))
  GUARD.start()
  yield
  assert GUARD.stop() == []


def test_the_adapter():
  # the top level and owner_config() import the standard library only: manager and
  # the resident owner import this module
  assert check_adapter(jetlink_adapter, heavy=('openpilot.common.params', 'openpilot.common.swaglog',
                                               'openpilot.cereal')) == []


def test_the_modeld_seam():
  assert check_seam('openpilot/selfdrive/modeld/modeld.py') == []


def test_the_fallback():
  hooks = ('should_run', 'status', 'reason', 'prepare', 'handed_over', 'state', 'shutdown_step', 'runtime',
           'swap_events')
  assert check_fallback(jetlink_adapter._Off('jetlink is missing'), hooks, link_on=jetlink_adapter.link_on) == []
