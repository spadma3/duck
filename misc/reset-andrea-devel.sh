git checkout master
git pull
git update-ref -m "resetting" refs/head/andrea-devel master
git push --force origin andrea-devel
