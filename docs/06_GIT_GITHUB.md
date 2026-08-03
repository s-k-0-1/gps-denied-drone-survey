# 06 — Git & GitHub (complete beginner's guide)

You have never used git before? Start at step 1 and follow along. Every command is meant to be
copy-pasted into your **WSL/Ubuntu terminal**.

---

## 0. What git and GitHub actually are

| | What it is |
|---|---|
| **git** | A program on *your computer* that records snapshots ("commits") of your project so you can see history and undo mistakes. |
| **GitHub** | A website that stores a copy of that history online, so others can see it and you have a backup. |

Workflow in one line:

```
edit files → git add (choose) → git commit (save snapshot) → git push (upload to GitHub)
```

---

## 1. Install git

```bash
sudo apt update && sudo apt install -y git
git --version        # e.g. git version 2.34.1
```

## 2. Tell git who you are

Use the same email as your GitHub account.

```bash
git config --global user.name  "Your Name"
git config --global user.email "you@example.com"
git config --global init.defaultBranch main
```

Check:

```bash
git config --global --list
```

---

## 3. Create the repository on GitHub

1. Go to <https://github.com> and sign in (create an account if needed).
2. Click **+** (top right) → **New repository**.
3. Fill in:
   - **Repository name:** `gps-denied-drone-survey`
   - **Description:** `Autonomous GPS-denied drone survey & feature localization`
   - **Public**
   - ⚠️ **Do NOT** tick "Add a README", ".gitignore" or "license" — we already have them.
4. Click **Create repository**.

Keep that page open — it shows your repository URL:

```
https://github.com/<your-username>/gps-denied-drone-survey.git
```

---

## 4. Create an access token (your "password" for pushing)

GitHub does not accept your account password from the terminal. You need a **Personal Access
Token (PAT)**.

1. GitHub → click your avatar → **Settings**
2. Left sidebar, scroll to the bottom → **Developer settings**
3. **Personal access tokens** → **Tokens (classic)** → **Generate new token (classic)**
4. Note: `laptop-wsl`; Expiration: 90 days (or longer)
5. Tick the **`repo`** scope
6. **Generate token** → **copy it now** (it is shown only once)

Save it somewhere safe — you will paste it as the *password* when git asks.

Make git remember it so you only paste once:

```bash
git config --global credential.helper store
```

---

## 5. Prepare your project folder

```bash
cd ~/advanced_matcher
```

### 5.1 Check what will be uploaded

The repo already contains a `.gitignore` that excludes large data folders (`drone_photos/`,
`results/`, `datasets/`, `results_archive/`, …). **This matters** — GitHub rejects files over
100 MB, and your results folders are huge.

Verify before doing anything else:

```bash
git init
git add -A
git status --short | head -40          # list of files that would be committed
```

You should see `.py`, `.md`, `.ino`, `.txt`, `base_station/…` — and **no** photos, `.tif`,
`.glb`, or `results_archive/`.

If you see data files, they are not being ignored — fix `.gitignore` before continuing:

```bash
git rm -r --cached .          # unstage everything (files stay on disk)
# edit .gitignore, then:
git add -A
git status --short
```

### 5.2 Check the total size

```bash
du -sh --exclude=.git --exclude=results --exclude=drone_photos \
       --exclude=datasets --exclude=results_archive .
```

Anything under ~50 MB is comfortable.

---

## 6. First commit and push

```bash
# 1. save the snapshot
git commit -m "Initial commit: ASCEND pipeline, dashboard, ESP32 firmware, docs"

# 2. name the branch
git branch -M main

# 3. connect to GitHub  (replace <your-username>)
git remote add origin https://github.com/<your-username>/gps-denied-drone-survey.git

# 4. upload
git push -u origin main
```

When prompted:
- **Username:** your GitHub username
- **Password:** paste the **token** from step 4 (not your account password)

Refresh the GitHub page — your code is live. 🎉

---

## 7. Everyday workflow (after any change)

```bash
git status                       # what changed?
git add -A                       # stage everything
git commit -m "Fix yellow mask threshold for bright arena"
git push
```

That's the whole loop. Commit often; each commit is a restore point.

### Writing good commit messages

| Good | Not useful |
|---|---|
| `Add VIO-based pair selection for stitching` | `update` |
| `Fix ADC clipping in pad voltage reading` | `changes` |
| `Document ESP32 wiring and charging states` | `asdf` |

---

## 8. Useful commands

| Command | What it does |
|---|---|
| `git status` | What has changed / what is staged |
| `git diff` | Exact line-by-line changes (not yet staged) |
| `git log --oneline -10` | Last 10 commits |
| `git add <file>` | Stage one file only |
| `git restore <file>` | Discard your changes to a file (⚠️ cannot be undone) |
| `git pull` | Download changes others pushed |
| `git clone <url>` | Copy a repo onto a new computer |
| `git remote -v` | Which GitHub repo this folder points to |

---

## 9. Setting it up on another computer

```bash
git clone https://github.com/<your-username>/gps-denied-drone-survey.git
cd gps-denied-drone-survey
pip install -r requirements.txt --break-system-packages
```

Then follow [05 — Setup](05_SETUP.md). Remember the data folders are *not* in git — copy
`drone_photos/` and `targets/` across separately.

---

## 10. Working with teammates

```bash
git pull                 # ALWAYS pull before you start editing
# … make your changes …
git add -A
git commit -m "Describe what you did"
git push
```

If two people edited the same lines, git reports a **conflict**. Open the file — you'll see:

```
<<<<<<< HEAD
your version
=======
their version
>>>>>>> origin/main
```

Delete the markers, keep the correct code, then:

```bash
git add <file>
git commit -m "Resolve merge conflict"
git push
```

Give collaborators access: GitHub repo → **Settings** → **Collaborators** → **Add people**.

---

## 11. Common problems

| Error / situation | Cause | Fix |
|---|---|---|
| `Authentication failed` | Used account password | Use the **token** from step 4 |
| `remote origin already exists` | Remote set twice | `git remote set-url origin <url>` |
| `Updates were rejected … fetch first` | GitHub has commits you don't | `git pull --rebase` then `git push` |
| `File … is 132 MB; exceeds GitHub's limit` | A data file got committed | See §12 below |
| `nothing to commit, working tree clean` | Nothing changed | Normal — carry on |
| `Please tell me who you are` | Identity not set | Redo step 2 |
| `fatal: not a git repository` | Wrong folder | `cd` into the project folder |

---

## 12. If you accidentally committed a huge file

Before pushing (easiest):

```bash
git rm -r --cached results/          # remove from the commit, keep on disk
echo "results/" >> .gitignore
git add -A
git commit -m "Remove large results folder from git"
```

If you already pushed it, the file stays in history and the repo remains bloated. The clean fix is
to start a fresh history:

```bash
rm -rf .git
git init
git add -A
git commit -m "Initial commit (clean history)"
git branch -M main
git remote add origin https://github.com/<your-username>/gps-denied-drone-survey.git
git push -f origin main        # -f overwrites the remote
```

⚠️ `push -f` erases the online history. Only do this on your own repo, early on.

---

## 13. Polishing the repo (optional but worth it)

**Add topics** — repo page → ⚙️ next to "About" → add `drone`, `computer-vision`, `dinov2`,
`ros`, `isro`, `photogrammetry`.

**Add screenshots to the README** — create a folder, commit images, and reference them:

```bash
mkdir -p docs/images
# copy in: orthomosaic.jpg, annotated_field.jpg, a dashboard screenshot
git add docs/images && git commit -m "Add result screenshots" && git push
```

```markdown
![Annotated arena](docs/images/annotated_field.jpg)
```

**Releases** — repo page → **Releases** → **Create a new release** → tag `v1.0` →
a short release note. Good for snapshotting a working version.

---

## 14. Complete copy-paste sequence

For your exact case, start to finish:

```bash
# one-time git setup
sudo apt install -y git
git config --global user.name  "Your Name"
git config --global user.email "you@example.com"
git config --global init.defaultBranch main
git config --global credential.helper store

# in the project folder
cd ~/advanced_matcher
git init
git add -A
git status --short | head -40        # ← confirm NO photos / results

git commit -m "Initial commit: ASCEND pipeline, dashboard, ESP32 firmware, docs"
git branch -M main
git remote add origin https://github.com/<your-username>/gps-denied-drone-survey.git
git push -u origin main              # username + TOKEN
```

Afterwards, whenever you change something:

```bash
git add -A && git commit -m "What I changed" && git push
```

---

**Back to:** [README](../README.md)
