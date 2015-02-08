# Git cheatsheet

This reference is non-exhaustive. For more information, I encourage you to look at Git's documentation or do some research online.


## General use
*These commands may modify local/remote state.*

**See changed files in your working directory**  
`git status`

**Download changes and directly merge/integrate into HEAD**  
`git pull`

**See changes to tracked files**  
`git diff`
OR
`git difftool`

**See changes to staged files**  
`git diff --cached`
OR
`git difftool --cached`

**Stage tracked/untracked file for commit**  
`git add <file>`

**Commit staged changes**  
`git commit -m "<commit message>"`

**Stage changes to tracked files and commit**  
`git commit -am "<commit message>"`

**Publish local changes on a remote**  
`git push`

**Stash your changes**  
`git stash`

**Get them back**  
`git stash pop`

**Discard local changes to file. Will lose work!**  
`git checkout HEAD <file>`

**Discard all changes to tracked files in the working tree since HEAD. Will lose work!**  
`git reset --hard HEAD`

**Fix conflicts**  
`git mergetool`

**Edit appropriate config file**  
`git config --global --edit`


## Advanced use
*These commands may modify local/remote state.*

**Rebasing (fetch and rebase with the origin server)**  
`git pull --rebase <remote name> <branch name>`

**Squash commits in place**  
`git reset --soft HEAD~3 && git commit`

**Rebase onto master and add a range of commits (P to R) on top**  
`git rebase --onto master O R`

**Move my new local commits on top of an older revision**  
`git rebase --onto 6eb22b394c7 origin/<branch name>`


## Branches
*These commands may modify local/remote state.*

**List all existing branches**  
`git branch`

**Switch to a branch**  
`git checkout <branch name>`

**Create a branch and switch to it**  
`git checkout –b feature/foo`

**Rename a local branch**  
`git branch -m [specify if not the current branch: <oldname>] <newname>`

**Merge a branch into master**  
First switch to master: `git checkout master`  
Then merge the feature branch: `git merge feature/foo`  

**Push a branch to remote and start tracking it (tracking means `git push and git pull will work for that branch)**  
`git push –u origin feature/foo`

**Deleting a branch after you’re done**  
`git branch –d feature/foo`

**Deleting a branch even if you didn't push stuff first (you will lose work!)**  
`git branch –D feature/foo`

**Delete the remote copy of a branch**  
`git push origin –delete feature/foo`

**Delete local and remote branch**  
`git branch -rd feature/foo`

**Delete remote after the fact**  
`git push origin --delete feature/foo`

**Weirder syntax for deleting remote**  
`git push origin :feature/foo`


## Show information
*These commands do not modify anything.*

**See changes not in remote**  
`git log origin/<branch name>..`

**Pretty log **  
`git log --oneline --graph --decorate`

**Show changes in a commit (no diff)**  
`git show --name-only e0193c3d872817ee7c2cae8c0960dafd5e3ed8c0`

**Who changed what and when in file**  
`git blame -w <file>`

**Diff against a specific commit**  
`git diff 6eb22b394c79a32ba6`

**Find file**  
`git ls-files '*/richtexteditor.js'`

**Show which files are being tracked**  
`git ls-tree -r [branch name] --name-only`

**Show origin**  
`git remote show origin`


## Extras
*These commands may modify local/remote state. You probably won't need to use these.*

**REALLY ignore stuff (even when switching branch)**  
`git update-index --skip-worktree <file>`

**Move stuff between repositories**  
`git fetch <remote-git-url> <sha-or-branch> && git cherry-pick FETCH_HEAD`

**If because of the skip-worktree things break**  
`git reset --hard HEAD`
OR
`git update-index  --really-refresh --no-assume-unchanged <file>`

**List files with skipped worktree**  
`git ls-files -v | grep ^S`

**Filtering list of branches**  
`git branch -vv --list bug*`

**Create patch for a specific commit**  
`git format-patch -n -1 -o <path> d59974a2db86`

**Create patch for a range of commit**  
`git format-patch -n  -o <path> d59974a2db86..a2345b4cc77`
