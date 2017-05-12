awk 'BEGIN {printf "%s\t %-10s\t %-15s\t %-17s\t %-9s\t %-10s\n", "number", "    ", "recieverNode", "sourceNodeMAC", "sequence", "delay(microsec)"}/Tx/ || /RxOk/{printf "%d\t %-15s \t %-15d\t %-15s\t %-10d\t %s\n", count++,$1, substr($3,11,3), substr($13,4,17), substr($16,11,3)-0, substr($2,4,5)}' Interface.tr>itiAll.txt
awk 'BEGIN {printf "%s\t %-10s\t %-15s\t %-17s\n", "number", "    ", "sourceNnode", "sourceNodeMAC"}/Tx/{printf "%d\t %-15s \t %-15d\t %-15s\n",count++, $1, substr($3,11,3), substr($13,4,17)}' Interface.tr>itiTx.txt

awk  'BEGIN {printf "%s\t %s\t %s\t %s\t %s\n", "Tx/Rx", "recieverNode", "sourceNnode", "sequence", "delay(microsec)"}FNR==NR {a[$4]=$3;next} {if ($4 in a) print $2,"\t","\t",$3,"\t","\t",a[$4],"\t","\t",$5,"\t","\t",$6}' itiTx.txt itiAll.txt > xit.txt

awk 'BEGIN {printf "%s\n%-19s\n", "LF topology", "Total received by each nodes"}/r/{ if ($3 == 0 && $2 == 1){countLF1++;}}{ if ($3 == 0 && $2 == 2){countLF2++;}}{ if ($3 == 0 && $2 == 3){countLF3++;}}END{printf "%-5s %d\n%-5s %d\n%-5s %d\n", "0 -> 1 : ", countLF1, "0 -> 2 : ", countLF2, "0 -> 3 : ", countLF3}' xit.txt>LF.txt

awk 'BEGIN {printf "%s\n%-19s\n", "PF topology", "Total received by each nodes"}/r/{ if ($3 == 0 && $2 == 1){countPF1++;}}{ if ($3 == 1 && $2 == 2){countPF2++;}}{ if ($3 == 2 && $2 == 3){countPF3++;}}END{printf "%-5s %d\n%-5s %d\n%-5s %d\n", "0 -> 1 : ", countPF1, "1 -> 2 : ", countPF2, "2 -> 3 : ", countPF3}' xit.txt>PF.txt

awk 'BEGIN {printf "%s\n%-19s\n", "PPF topology", "Total received by each nodes"}/r/{ if ($3 == 0 && $2 == 1){countPPF1++;}}{ if ($3 == 0 && $2 == 2){countPPF2++;}}{ if ($3 == 1 && $2 == 3){countPPF3++;}}END{printf "%-5s %d\n%-5s %d\n%-5s %d\n", "0 -> 1 : ", countPPF1, "0 -> 2 : ", countPPF2, "1 -> 3 : ", countPPF3}' xit.txt>PPF.txt

awk 'BEGIN {printf "%s", "number of transmits (divide by numberofvehicles+numberofvehiclesCity)"}/t/{countT++;}END{printf ": %d\n", countT;}' xit.txt>T.txt
