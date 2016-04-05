# !/bin/bash
date
echo "================================================================="
echo "start execute $0"
clpru -I=/home/roger/PRUSS/ti-cgt-pru_2.1.2/include pwmpru1.c -z lnk-am33xx.cmd -o pwmpru1.elf -i=/home/roger/PRUSS/ti-cgt-pru_2.1.2/lib
echo "end execute $0"
echo "================================================================="
