"""Keep the documentation's file:line references honest.

The README's "Build your own controller" section is a table of links with line
ranges. Those rot the moment the code they point at moves, and a tutorial with
links into the wrong function is worse than no links. This walks every markdown
file in the package and checks each link.

Nothing here needs a robot, a build, or a ROS graph.
"""

import os
import re

import pytest

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

LINK = re.compile(r"\[([^\]]+)\]\(([^)]+)\)")
LINE_ANCHOR = re.compile(r"^L(\d+)(?:-L(\d+))?$")
HEADING = re.compile(r"^#+\s+(.+)$", re.M)


def markdown_files():
    found = []
    for dirpath, dirnames, filenames in os.walk(PACKAGE_ROOT):
        # Skip caches and VCS directories; .pytest_cache ships its own README.
        dirnames[:] = [
            d for d in dirnames if d != "__pycache__" and not d.startswith(".")
        ]
        found.extend(
            os.path.join(dirpath, f) for f in filenames if f.endswith(".md")
        )
    return sorted(found)


def slugify(heading):
    """GitHub's anchor slug for a heading."""
    return re.sub(r"[^a-z0-9]+", "-", heading.lower()).strip("-")


def links_in(path):
    for label, target in LINK.findall(open(path, encoding="utf-8").read()):
        if not target.startswith(("http://", "https://", "mailto:")):
            yield label, target


@pytest.mark.parametrize("doc", markdown_files(), ids=os.path.basename)
def test_links_resolve(doc):
    """Every relative link points at a file that exists."""
    base = os.path.dirname(doc)
    for label, target in links_in(doc):
        path = target.partition("#")[0]
        if not path:
            continue
        resolved = os.path.normpath(os.path.join(base, path))
        assert os.path.exists(resolved), f"{label} -> {target} does not exist"


@pytest.mark.parametrize("doc", markdown_files(), ids=os.path.basename)
def test_line_anchors_are_within_the_file(doc):
    """Line ranges do not run past the end of the file they point at."""
    base = os.path.dirname(doc)
    for label, target in links_in(doc):
        path, _, fragment = target.partition("#")
        match = LINE_ANCHOR.match(fragment) if fragment else None
        if not match or not path:
            continue
        resolved = os.path.normpath(os.path.join(base, path))
        if not os.path.exists(resolved):
            # Covered by test_links_resolve; the controller lives in a
            # submodule that may not be checked out.
            continue
        total = len(open(resolved, encoding="utf-8", errors="replace").read().split("\n"))
        start = int(match.group(1))
        end = int(match.group(2) or match.group(1))
        assert 1 <= start <= end <= total, (
            f"{label} -> {target} is outside {os.path.basename(resolved)} "
            f"({total} lines)"
        )


@pytest.mark.parametrize("doc", markdown_files(), ids=os.path.basename)
def test_link_labels_match_their_anchors(doc):
    """A label reading `file.py:160-173` must point at exactly those lines.

    This is what catches a stale reference: the code moves, someone updates the
    anchor, and the human-readable label is left behind (or the reverse).
    """
    for label, target in links_in(doc):
        path, _, fragment = target.partition("#")
        anchor = LINE_ANCHOR.match(fragment) if fragment else None
        # Labels are written as `file.py:160-173`, backticks included, so strip
        # the formatting before matching -- otherwise the $ anchor never binds
        # and this check quietly passes on everything.
        labelled = re.search(r":(\d+)(?:-(\d+))?$", label.strip().strip("`"))
        if not anchor or not labelled:
            continue
        anchor_range = (int(anchor.group(1)), int(anchor.group(2) or anchor.group(1)))
        label_range = (
            int(labelled.group(1)),
            int(labelled.group(2) or labelled.group(1)),
        )
        assert label_range == anchor_range, (
            f"in {os.path.basename(doc)}: label says {label_range}, "
            f"link points at {anchor_range} ({target})"
        )


@pytest.mark.parametrize("doc", markdown_files(), ids=os.path.basename)
def test_heading_anchors_exist(doc):
    """A #section link names a heading that is actually there."""
    base = os.path.dirname(doc)
    for label, target in links_in(doc):
        path, _, fragment = target.partition("#")
        if not fragment or LINE_ANCHOR.match(fragment):
            continue
        resolved = os.path.normpath(os.path.join(base, path)) if path else doc
        if not resolved.endswith(".md") or not os.path.exists(resolved):
            continue
        slugs = {
            slugify(h) for h in HEADING.findall(open(resolved, encoding="utf-8").read())
        }
        assert fragment in slugs, (
            f"{label} -> {target}: no heading in "
            f"{os.path.basename(resolved)} slugs to '{fragment}'"
        )


def test_the_index_points_at_definitions_not_blank_lines():
    """The first line of every indexed range should be real code.

    A range that opens on a blank line usually means the reference drifted.
    """
    readme = os.path.join(PACKAGE_ROOT, "README.md")
    offenders = []
    for label, target in links_in(readme):
        path, _, fragment = target.partition("#")
        match = LINE_ANCHOR.match(fragment) if fragment else None
        if not match or not path:
            continue
        resolved = os.path.normpath(os.path.join(PACKAGE_ROOT, path))
        if not os.path.exists(resolved):
            continue
        lines = open(resolved, encoding="utf-8", errors="replace").read().split("\n")
        start = int(match.group(1))
        if start > len(lines):
            offenders.append(f"{label} -> {target} starts past the end of the file")
            continue
        if not lines[start - 1].strip():
            offenders.append(f"{label} -> {target} starts on a blank line")
    assert not offenders, "\n".join(offenders)
