---
title: Introduction to Ansible
date: 30 Jan 2021
image: "https://upload.wikimedia.org/wikipedia/commons/2/24/Ansible_logo.svg"
---

Ansible is an agentless automation scripting language (using yaml files and back end is python) that uses SSH to access servers and execute commands. Owned now by Red Hat, but still open source. 

- *Optional*: setup virtualenv  
    - `python3 -m venv ~/ansi`
    - `source ~/ansi/bin/activate`
    - `pip install -U pip setuptools wheel`
- Install: `pip install ansible`

## Usage

Simple ping example of all the servers listed in the inventory file:

- Execute: `ansible -i inventory multi -m ping`
    - `-i inventory`: inventory file shown below, could be called anything
    - `multi`: is the group which contains the sub groups `pis` and `ubuntu`
    - `-m ping`: calls the ping module

```
# Simple stupid inventory example
[pis]
raspberrypi.local

[pis:vars]
ansible_user=pi

[ubuntu]
ubuntu.local

[ubuntu:vars]
ansible_user=ubuntu

[multi:children]
pis
ubuntu
```

To see what ansible knows about your system:

```
ansible -i inventory -u pi raspberrypi.local -m setup

# if you setup a ansible.cfg with NVENTORY = inventory.yml
ansible -u pi -m setup raspberrypi.local
ansible -u pi -m ping raspberrypi.local
```

A large dictionary of information will print to the screen. This info can also be 
used dynamically.

### Other Useful Features

| Switch | Notes |
|--------|-------|
| `-b`   | Elevate to root user  |
| `-i <file>` | Use an inventory file which contains info about servers |
| `-u <user>` | Specify what user to login as |

> **WARNING:** If you get errors about `(publickey, password)`, make sure you have used
`ssh-copy-id` so you can access the computer with `ssh` but without having to use
a password.

## Ansible Vault

`ansible-vault encrypt|decrypt|rekey|edit|view <file>`

```yaml
---
API_KEY: 123456789abcdefg
```

```
(ansi) kevin@Logan ansible % ansible-vault encrypt key.yml
New Vault password: 
Confirm New Vault password: 
Encryption successful

(ansi) kevin@Logan ansible % cat key.yml 
$ANSIBLE_VAULT;1.1;AES256
63663662373337306632386463376133383933643736343831363535306565393262343461353665
6538636562613161303033383465333533353665633161620a366336623666323237313437323861
65356634366565623539633031623439373534653565336433336232653237396361323938333338
3364353734356437320a333939363732653061343463663530626665643836626137346339616532
62663363393264363834373763643330323433333633636337383132323236366435
```

```
(ansi) kevin@Logan ansible % ansible-vault view key.yml   
Vault password: 
---
API_KEY: 123456789abcdefg
```

## Playbooks


```yaml
---
- name: Debug string
  hosts: all
  gather_facts: false
  
  # scripting variables that apply to each task also
  vars:
    bob: "hello there bob"
    
  # global env variable that apply to each task also
  environment:
    LANG=fr_FR.UTF-8
    
  - debug: msg="The var is {{ bob }}"
    
  # import another playbook
  - import_playbook: apt.yml
  
  # include playbook, allows dynamic vars
  - include_playbook: play2.yml
    
  tasks:
    - name: Doing something cool using some_module
      some_module
      register: foo
      
    - debug: var=foo
    
    - debug: var=foo.key
    
    - debug: var=foo["key"]
```

For modularized playbooks, it can be confusing what higher level playbook
called a lower level playbook. Roles helps with this:

```yaml
roles:
    - python
```

Now, every `TASK` that prints, will pre-pend `python` onto the output.

### Var File

You can reference variables in a yaml file

```yaml
---
# playbook that doesn't have to change, you can
# just update vars.yml now
pre_tasks:
  - name: load variable files
    include_vars: "{{ item }}"
    with_first_found:
      - "{{ ansible_os_family }}.yml"
      - "default.yml"
```

```yaml
---
# default.yml
ip_addr: 10.100.100.10
count: 25
```

### Modules Template

So I got tired of re-writing this over and over, all of the module examples should 
have something like this added to them if they don't:

```yaml
---
- name: Debug string
  hosts: all
    
  tasks:
    - name: Doing something cool using some_module
      some_module
```

### APT

Can make more generic by using the `package` module rather than the `apt` module.

```yaml
---
- name: APT
  hosts: all
  tasks:
    - name: Pass options to dpkg on run
      apt:
        upgrade: dist
        update_cache: yes

    - name: Remove useless packages from the cache
      apt:
        autoclean: yes

    - name: Remove dependencies that are no longer required
      apt:
        autoremove: yes
```

### PIP

```yaml
---
- name: PIP
  hosts: all
  
  tasks:
    - pip:
        name:
          - django>1.11.0,<1.12.0
          - bottle>0.10,<0.20,!=0.11
```


### Commands and Shell

```yaml
---
- name: Commands and Shell
  hosts: all
  
  tasks:
    - name: Run shell
      shell: |
          apt install -y python3
          which python
    - name: Run command
      command: service httpd status
    - name: Run another command
      command: >
        service httpd status
          
```

### Services

```yaml
- name: Start httpd
  service:
    name: httpd
    state: started
    enable: true
```

### Homebrew

```yaml
# Update homebrew and upgrade all packages
- homebrew:
    update_homebrew: yes
    upgrade_all: yes
    
- homebrew:
    name: foo
    state: present
    install_options: with-baz,enable-debug
    update_homebrew: yes
```

### Env with Lineinfile

`become: false` turns off root priv and uses the login user.

```yaml
- name: Add env var to remote server bash env
  lineinfile:
    dest: "~/.bash_profile"
    regexp: '^ENV_VAR='
    line: 'ENV_VAR=value'
  become: false
  
- name: Get the vlue of the remote env var
  shell: 'source ~/.bash_profile && echo $ENV_VAR'
  register: foo
  
- debug: msg="The var is {{ foo.stdout }}"
```

# References

- Ansible docs: [List of Modules](https://docs.ansible.com/ansible/2.9/modules/list_of_packaging_modules.html)
