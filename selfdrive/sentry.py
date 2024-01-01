"""Install exception handler for process crash."""
import sentry_sdk
from enum import Enum
from sentry_sdk.integrations.threading import ThreadingIntegration

from openpilot.common.params import Params
from openpilot.selfdrive.athena.registration import is_registered_device
from openpilot.system.hardware import HARDWARE, PC
from openpilot.common.swaglog import cloudlog
from openpilot.system.version import get_branch, get_commit, get_origin, get_version, \
                              is_comma_remote, is_dirty, is_tested_branch


dongle = Params().get("DongleId", encoding='utf-8')

class SentryProject(Enum):
  # All sentry data goes to AF. comma doesn't want fork data anyway
  # python project
  SELFDRIVE = "https://741d934bafba6e550d348f17c32dc1dc@o1269754.ingest.sentry.io/4506477324533760"
  # native project
  SELFDRIVE_NATIVE = "https://e2e617f1ccebd58ed7aa8fbb15ed5b95@o1269754.ingest.sentry.io/4506477325582336"
  AF = "https://21654306f32a4cc29d283e7e068cf27c@o1269754.ingest.sentry.io/6460006"

def capture_message(msg: str, data: str, file: str) -> None:
  try:
    # Encode bytes to base64 for attachment
    attachment = str(data).encode()
    # Add attachment to the current scope
    # with sentry_sdk.start_transaction() as transaction:
    with sentry_sdk.configure_scope() as scope:
      scope.add_attachment(
          bytes=attachment,
          filename=f'{dongle}_{file}.txt',
          content_type="text/plain"
      )
    sentry_sdk.capture_message(f'{dongle}: {msg}')
    sentry_sdk.flush()  # https://github.com/getsentry/sentry-python/issues/291
  except Exception as e:
    print(e)

def report_tombstone(fn: str, message: str, contents: str) -> None:
  cloudlog.error({'tombstone': message})

  with sentry_sdk.configure_scope() as scope:
    scope.set_extra("tombstone_fn", fn)
    scope.set_extra("tombstone", contents)
    sentry_sdk.capture_message(message=message)
    sentry_sdk.flush()


def capture_exception(*args, **kwargs) -> None:
  cloudlog.error("crash", exc_info=kwargs.get('exc_info', 1))

  try:
    sentry_sdk.capture_exception(*args, **kwargs)
    sentry_sdk.flush()  # https://github.com/getsentry/sentry-python/issues/291
  except Exception:
    cloudlog.exception("sentry exception")


def set_tag(key: str, value: str) -> None:
  sentry_sdk.set_tag(key, value)


def init(project: SentryProject) -> bool:
  # forks like to mess with this, so double check
  af_remote = "csouers" in get_origin(default="")
  comma_remote = af_remote or (is_comma_remote() and "commaai" in get_origin(default=""))
  if not comma_remote or not is_registered_device() or PC:
    print('sentry: device or remote not allowed')
    return False

  env = "release" if is_tested_branch() else "master"
  dongle_id = Params().get("DongleId", encoding='utf-8')

  integrations = []
  if project == SentryProject.SELFDRIVE:
    integrations.append(ThreadingIntegration(propagate_hub=True))
  else:
    sentry_sdk.utils.MAX_STRING_LENGTH = 8192

  sentry_sdk.init(project.value,
                  default_integrations=False,
                  release=get_version(),
                  integrations=integrations,
                  traces_sample_rate=1.0,
                  environment=env)

  sentry_sdk.set_user({"id": dongle_id})
  sentry_sdk.set_tag("dirty", is_dirty())
  sentry_sdk.set_tag("origin", get_origin())
  sentry_sdk.set_tag("branch", get_branch())
  sentry_sdk.set_tag("commit", get_commit())
  sentry_sdk.set_tag("device", HARDWARE.get_device_type())

  if project == SentryProject.SELFDRIVE:
    sentry_sdk.Hub.current.start_session()

  return True
