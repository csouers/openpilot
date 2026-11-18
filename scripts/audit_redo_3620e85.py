#!/usr/bin/env python3
"""Read-only, pinned-reference checks for the 3620e85 merge reconstruction."""
import ast
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ORIGINAL = "e8882b4bd"
FORK = "d67bd687f"
UPSTREAM = "b1a662033"
MERGE = "8f7c829dd"
RENDERER = "openpilot/selfdrive/ui/mici/onroad/model_renderer.py"
BASE_RENDERER = "openpilot/selfdrive/ui/onroad/model_renderer.py"
EXPECTED_DIFFERENCES = {
  RENDERER,
  "openpilot/selfdrive/ui/mici/onroad/augmented_road_view.py",
  "openpilot/selfdrive/ui/tests/test_mici_upstream_geometry.py",
  "docs/merge-redo-3620e85.md",
  "scripts/audit_redo_3620e85.py",
}


def git(*args):
  return subprocess.check_output(["git", "-C", str(ROOT), *args], text=True).strip()


def source(ref, path):
  return subprocess.check_output(["git", "-C", str(ROOT), "show", f"{ref}:{path}"], text=True)


def methods(text):
  cls = next(n for n in ast.parse(text).body if isinstance(n, ast.ClassDef) and n.name == "ModelRenderer")
  return {n.name: n for n in cls.body if isinstance(n, ast.FunctionDef)}


def dump(node):
  return ast.dump(node, include_attributes=False)


def visibility_block(method):
  for i, node in enumerate(method.body):
    if isinstance(node, ast.Assign) and any(
      isinstance(t, ast.Attribute) and t.attr == "_visible" for t in node.targets
    ):
      return [dump(n) for n in method.body[i:i + 2]]
  raise AssertionError("missing road visibility block")


def main():
  assert git("show", "-s", "--format=%P", MERGE).split() == [
    git("rev-parse", FORK), git("rev-parse", UPSTREAM),
  ]
  print("PASS: replacement merge has the exact original parents")

  base = git("merge-base", FORK, UPSTREAM)
  fork_paths = set(git("diff", "--name-only", base, FORK).splitlines())
  upstream_paths = set(git("diff", "--name-only", base, UPSTREAM).splitlines())
  untouched = fork_paths - upstream_paths
  for path in untouched:
    assert git("rev-parse", f"{FORK}:{path}") == git("rev-parse", f"{MERGE}:{path}"), path
  print(f"PASS: all {len(untouched)} non-overlapping fork files retained byte-for-byte in replacement merge")
  print(f"REVIEW: {len(fork_paths & upstream_paths)} overlapping paths; see behavior audit in docs")

  actual = set(git("diff", "--name-only", ORIGINAL, "HEAD").splitlines())
  assert actual == EXPECTED_DIFFERENCES, actual ^ EXPECTED_DIFFERENCES
  print("PASS: every final-tree difference from original HEAD is explicitly allowlisted")

  assert source("HEAD", BASE_RENDERER) == source(UPSTREAM, BASE_RENDERER)
  current, upstream = methods(source("HEAD", RENDERER)), methods(source(UPSTREAM, RENDERER))
  for name in ("_draw_offset", "_get_ll_color", "_draw_lane_lines", "_draw_path"):
    assert dump(current[name]) == dump(upstream[name]), name
  for name in ("_update_model", "_update_path_gradient", "_get_path_length_idx",
               "_map_line_to_polygon", "_hsla_to_color", "_draw_steer_path"):
    assert name not in current, name
  assert visibility_block(current["_render"]) == visibility_block(upstream["_render"])
  print("PASS: upstream base renderer, mici road draw/color methods and disengagement visibility")

  for path, expected in (("panda", "d68f2ec7346851e16574cd598aaf931e03c8764e"),
                         ("opendbc_repo", "7c68585ed36b1ce5535a411110d7b9c6ce803f1f")):
    assert git("rev-parse", f"HEAD:{path}") == expected
    assert git("rev-parse", f"{ORIGINAL}:{path}") == expected
  print("PASS: fork submodule pointers unchanged from original HEAD")

  for path in (ROOT / "openpilot/selfdrive/ui").rglob("*.py"):
    for node in ast.walk(ast.parse(path.read_text())):
      if isinstance(node, ast.Attribute):
        assert node.attr != "started_frame", str(path)
  print("PASS: no obsolete public started_frame attribute reads in UI")


if __name__ == "__main__":
  main()
