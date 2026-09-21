"""
LibreLane plugin: draw Metal4 PDN stripes over the IHP SRAM's real power
columns, since the default generator fails on this macro's split-column
layout (see odb_sram_stripes.py for why).

LibreLane auto-imports every module on the Python path whose name starts
with ``librelane_plugin_`` (librelane/plugins.py). ``python -m librelane``
and the Tiny Tapeout GDS action both run from the repository root, so this
file is found with no install step. src/config.json inserts the step right
after the default PDN generator:

    "meta": {
      "substituting_steps": {
        "+OpenROAD.GeneratePDN": "Project.ExtendSramPowerStripes"
      }
    }

so the normal grid is built everywhere first, and this step only fills in
the region the default generator left empty.
"""
import os

from librelane.config import Variable
from librelane.steps import Step
from librelane.steps.odb import OdbpyStep

HERE = os.path.dirname(os.path.abspath(__file__))


@Step.factory.register()
class ExtendSramPowerStripes(OdbpyStep):
    id = "Project.ExtendSramPowerStripes"
    name = "Extend Power Stripes Over IHP SRAM"

    config_vars = [
        Variable(
            "SRAM_PDN_INSTANCE",
            str,
            "Hierarchical instance name of the SRAM macro to power, e.g. "
            "mem.sram. Must match a MACROS.<macro>.instances key in "
            "config.json.",
        ),
        Variable(
            "SRAM_PDN_LEF_RELPATH",
            str,
            "Path to the SRAM macro's LEF file, relative to $PDK_ROOT/$PDK "
            "(same file MACROS.<macro>.lef points at via pdk_dir::). A "
            "typed LibreLane Path variable is deliberately not used here: "
            "pdk_dir:: resolution needs librelane.config.Path, whose real "
            "import location isn't documented and turned out to be wrong "
            "on the first attempt (ImportError). Computing the path in "
            "plain Python from $PDK_ROOT avoids depending on it.",
        ),
        Variable(
            "SRAM_PDN_LAYER",
            str,
            "PDN layer the macro's power pins are drawn on.",
            default="Metal4",
        ),
    ]

    def get_script_path(self):
        return os.path.join(HERE, "odb_sram_stripes.py")

    def _lookup(self, key):
        """PDK_ROOT/PDK come from the resolved config, not the environment.

        The step's command is built in a context where those environment
        variables are not set (KeyError: 'PDK'); LibreLane carries them as
        ordinary config variables instead. Environment is kept only as a
        fallback.
        """
        try:
            value = self.config[key]
        except Exception:
            value = None
        return value or os.environ.get(key)

    def get_command(self):
        pdk_root = self._lookup("PDK_ROOT")
        pdk = self._lookup("PDK")
        if not pdk_root or not pdk:
            raise RuntimeError(
                f"cannot locate the PDK: PDK_ROOT={pdk_root!r} PDK={pdk!r}"
            )
        lef = os.path.join(
            str(pdk_root), str(pdk), self.config["SRAM_PDN_LEF_RELPATH"]
        )
        return super().get_command() + [
            "--instance", self.config["SRAM_PDN_INSTANCE"],
            "--lef", lef,
            "--layer", self.config["SRAM_PDN_LAYER"],
        ]
