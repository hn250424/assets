1. Install required packages.
```
sudo apt update  
sudo apt install -y build-essential libssl-dev libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm libffi-dev zlib1g-dev liblzma-dev
```

2. Install pyenv.
```
curl https://pyenv.run | bash
```

3. Env.
#### vi ~/.bashrc
```
export PATH="$HOME/.pyenv/bin:$PATH"  
eval "$(pyenv init --path)"  
eval "$(pyenv init -)"  
eval "$(pyenv virtualenv-init -)"  
```

4. Apply changes.
`source ~/.bashrc`

5. Check pyenv version.
`pyenv --version`

6. List installed Python versions.
`pyenv versions`

7. Show the currently activated version for this directory.
`pyenv version`

8. Install the specific Python version.
`pyenv install 3.10.12`

9. Create a virtual environment.
`pyenv virtualenv 3.10.12 my-pyenv`

10. Activate the environment.
`pyenv activate my-pyenv`

11. Deactivate the environment.
`pyenv deactivate`

12. Install requirements in the activate environment.
`pip install -r requirements.txt`

13. Export requirements from the activate environment.
`pip freeze > requirements.txt`