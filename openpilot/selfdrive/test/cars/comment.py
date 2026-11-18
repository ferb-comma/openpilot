import json
import os
import re
import tempfile

import requests

from openpilot.tools.lib.github_utils import GithubUtils

MARKER = "_cars_"


def shard_block(lines: list[str], summary: str) -> str:
  return "\n".join([*lines, summary]) + "\n"


def comment_body(shard_texts: dict[int, str], shards: int, commit: str) -> str:
  lines = []
  passed = failed = gaps = 0
  for i in sorted(shard_texts):
    for line in shard_texts[i].splitlines():
      passed += line.startswith("✅")
      failed += line.startswith("❌")
      if m := re.search(r"(\d+) gaps", line):
        gaps = int(m.group(1))
      if line.startswith(("✅", "❌", "    repeat:")):
        lines.append(line)

  header = f"cars: {passed} passed, {failed} failed"
  running = shards - len(shard_texts)
  if running:
    header += f" ({running} of {shards} shards still running)"
  return f"<!-- {MARKER} -->\n{header}\n\ncommit {commit[:7]}, {gaps} gaps (not driven)\n\n```\n" + "\n".join(lines) + "\n```"


def post_comment(block: str, shard: int, shards: int, commit: str) -> None:
  api_token, data_token, branch = os.getenv("GITHUB_COMMENTS_TOKEN"), os.getenv("CI_ARTIFACTS_TOKEN"), os.getenv("GIT_BRANCH")
  if not (api_token and data_token and branch):
    return
  try:
    github = GithubUtils(api_token, data_token)
    github.get_pr_number(branch)
    bucket = f"cars_{commit}"
    with tempfile.TemporaryDirectory() as tmp:
      path = os.path.join(tmp, f"shard{shard}.txt")
      with open(path, "w") as f:
        f.write(block)
      github.upload_files(bucket, [(f"shard{shard}.txt", path)])

    # a shard that has not uploaded yet is still running
    texts = {}
    for i in range(shards):
      if github.get_file_sha(bucket, f"shard{i}.txt"):
        texts[i] = requests.get(github.get_file_url(bucket, f"shard{i}.txt"), timeout=30).text
    # comment_on_pr puts the body in a json string as is
    body = json.dumps(comment_body(texts, shards, commit))[1:-1]
    github.comment_on_pr(body, branch, "commaci-public", True, comment_marker=MARKER)
  except Exception as e:
    print(f"comment not posted: {e}", flush=True)
