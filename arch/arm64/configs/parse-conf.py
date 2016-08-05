import re
import os
import string
import sys

# this is a simple parser for parsing kernel defconfig file
# 1. get the diffs of two given defconfig. 
# 2. merge the 1's diffs to a given defconfig. 
# you can use the function 1 for common use, and the function 2's logical can be use to convert project defconfig to per-defconfig
# base on the orignal defconfig and perf-defconfig


cnfitem1 = {}# confitem line, format: conf:value   maybe[y/n/m/"-perf"/"is not set"...]
cnfitem2 = {}

cnfdiff  = {}

#cnfitem3 = {} #used for merge

diff_file = 'diffs.txt'

#parsefile
def parsefile(f1, f2):
    file = open(f1,"r")
    total_1 = 0
    for line in file.readlines():
        line = line.strip()
        #print(line)
        if line == '':
            continue
        tag_conf = line.find('CONFIG')
        tag_equ  = line.find('=')

        if tag_conf == 0 and tag_equ != -1:
            conf_name = line.split('=')[0]
            conf_name = conf_name.strip()
            attr_name = line.split('=')[1]
            attr_name = attr_name.strip()
            cnfitem1[conf_name] = attr_name
            total_1 += 1
        elif tag_conf > 0 and line[0] == '#':
            line = line[1:]
            line = line.strip()
            tag_spa = line.find(' ')
            conf_name = line[0:tag_spa].strip()
            attr_name = line[tag_spa:].strip()
            cnfitem1[conf_name] = attr_name
            total_1 += 1
        else:
            print("file1 format error")
            return -1
    file.close()

    file = open(f2,"r")
    total_2 = 0
    for line in file.readlines():
        line = line.strip()
        if line == '':
            continue
        #print(line)
        tag_conf = line.find('CONFIG')
        tag_equ  = line.find('=')

        if tag_conf == 0 and tag_equ != -1:
            conf_name = line.split('=')[0]
            conf_name = conf_name.strip()
            attr_name = line.split('=')[1]
            attr_name = attr_name.strip()
            cnfitem2[conf_name] = attr_name
            total_2 += 1
        elif tag_conf > 0 and line[0] == '#':
            line = line[1:]
            line = line.strip()
            tag_spa = line.find(' ')
            conf_name = line[0:tag_spa].strip()
            attr_name = line[tag_spa:].strip()
            cnfitem2[conf_name] = attr_name
            total_2 += 1
        else:
            print("file2 format error")
            return -1
    file.close()

    return [total_1, total_2] 

#getdiff
def getdiff(t1, t2):

    for key in cnfitem1.keys():
        val1 = cnfitem1.get(key)
        val2 = cnfitem2.get(key)
        if val1 == val2 :
            continue#cnfdiff[key] = 0; if you want print the same conf, you can put it in cfdiff,too
        elif val2 == None:
            cnfdiff[key] = 1
        else:
            cnfdiff[key] = 3
    
    for key in cnfitem2.keys():
        val1 = cnfitem1.get(key)
        val2 = cnfitem2.get(key)
        if val1 == val2 :
            continue#cnfdiff[key] = 0; if you want print the same conf, you can put it in cfdiff,too
        elif val1 == None:
            cnfdiff[key] = 2
        else:
            cnfdiff[key] = 3


#do 3 loop...
    file = open(diff_file,"w")
    file.write("Auto generated file:")

    file.write("\n")
    file.write("\n")
    file.write("Items only in file1:\n")
    total_dif = 0
    for key in cnfdiff.keys():
        value = cnfdiff.get(key)
        if value == 1:
            value = cnfitem1.get(key)
            file.write(str(key))
            file.write('    ')
            file.write(str(value))
            file.write("\n")
            total_dif += 1
        else:
            continue
    file.write('total: %d' %(total_dif))

    file.write("\n")
    file.write("\n")
    file.write("Items only in file2:\n")
    total_dif = 0
    for key in cnfdiff.keys():
        value = cnfdiff.get(key)
        if value == 2:
            value = cnfitem2.get(key)
            file.write(str(key))
            file.write('    ')
            file.write(str(value))
            file.write("\n")
            total_dif += 1
        else:
            continue
    file.write('total: %d' %(total_dif))

    file.write("\n")
    file.write("\n")
    file.write("Items both in two files:\n")
    total_dif = 0
    for key in cnfdiff.keys():
        value = cnfdiff.get(key)
        if value == 3:
            value1 = cnfitem1.get(key)
            value2 = cnfitem2.get(key)
            file.write('++++')
            file.write(str(key))
            file.write('    ')
            file.write(str(value1))
            file.write("\n")
            file.write('----')
            file.write(str(key))
            file.write('    ')
            file.write(str(value2))
            file.write("\n")
            file.write("\n")
            total_dif += 1
        else:
            continue
    file.write('total: %d' %(total_dif))
    file.write('\n')

    file.close()

#mergediff
#we just add it 
def mergediff(f):

    f_m = "%s_merge" %(f)

    file = open(f,"r")
    file_merge = open(f_m, "w")

    total_chg = 0
    for line in file.readlines():
        pure_line = line
        line = line.strip()
        if line == '':
            continue
        #print(line)
        tag_conf = line.find('CONFIG')
        tag_equ  = line.find('=')

        if tag_conf == 0 and tag_equ != -1:
            conf_name = line.split('=')[0]
            conf_name = conf_name.strip()
            attr_name = line.split('=')[1]
            attr_name = attr_name.strip()

            diff_tag = cnfdiff.get(conf_name)
            
            if diff_tag == None:
                file_merge.write(pure_line)
                #file_merge.write('\n')
            elif diff_tag == 1:
                print ('DEL key:%s  value:%s' %(conf_name, attr_name))
                cnfdiff.pop(conf_name)
                total_chg += 1
            else: #diff_tag == 2 or diff_tag == 3
                diff_value = cnfitem2.get(conf_name)
                cnfdiff.pop(conf_name)
                print ('CHG key:%s  oldvalue:%s  newvalue:%s' %(conf_name, attr_name, diff_value))
                total_chg += 1
                if diff_value.find('not') >= 0:
                    file_merge.write('# ')
                    file_merge.write(str(conf_name))
                    file_merge.write(' ')
                    file_merge.write(str(diff_value))
                    file_merge.write('\n')
                else:
                    file_merge.write(str(conf_name))
                    file_merge.write('=')
                    file_merge.write(str(diff_value))
                    file_merge.write('\n')

        elif tag_conf > 0 and line[0] == '#':
            line = line[1:]
            line = line.strip()
            tag_spa = line.find(' ')
            conf_name = line[0:tag_spa].strip()
            attr_name = line[tag_spa:].strip()
 
            diff_tag = cnfdiff.get(conf_name)
            if diff_tag == None:
                file_merge.write(pure_line)
                #file_merge.write('\n')
            elif diff_tag == 1:
                cnfdiff.pop(conf_name)
                print ('DEL key:%s  value:%s' %(conf_name, attr_name))
                total_chg += 1
            else: #diff_tag == 2 or diff_tag == 3
                cnfdiff.pop(conf_name)
                diff_value = cnfitem2.get(conf_name)
                print ('CHG key:%s  oldvalue:%s  newvalue:%s' %(conf_name, attr_name, diff_value))
                total_chg += 1
                if diff_value.find('not') >= 0:
                    file_merge.write('# ')
                    file_merge.write(str(conf_name))
                    file_merge.write(' ')
                    file_merge.write(str(diff_value))
                    file_merge.write('\n')
                else:
                    file_merge.write(str(conf_name))
                    file_merge.write('=')
                    file_merge.write(str(diff_value))
                    file_merge.write('\n')
        else:
            print("merge file format error")
            return -1
    file.close()

    #clear the cnfdiff
    for key in cnfdiff.keys():
        value = cnfdiff.get(key)
        if value == 1:
            continue
        else:
            diff_value = cnfitem2.get(key)
            total_chg += 1
            print ('ADD key:%s  value:%s' %(key, diff_value))
            if diff_value.find('not') >= 0:
                file_merge.write('# ')
                file_merge.write(str(key))
                file_merge.write(' ')
                file_merge.write(str(diff_value))
                file_merge.write('\n')
            else:
                file_merge.write(str(key))
                file_merge.write('=')
                file_merge.write(str(diff_value))
                file_merge.write('\n')
    
    file_merge.close()
    return total_chg

#=======================================================================================================================
#main code

if len(sys.argv) == 4:
    merge_file = sys.argv[3]
    print ('Get diffs and also merge diffs')
    print ('Step1: compare two files: %s and %s' %(sys.argv[1], sys.argv[2]))
    [t1, t2]=parsefile(sys.argv[1],sys.argv[2])
    getdiff(t1, t2)
    print ('Step2: generate diffs to file %s, you can check it later' %(diff_file))
    print ('Step3: merge diffs[%s] to %s, and get a new merged file %s_merge' %(diff_file, merge_file, merge_file))
    print ('Merge Statistics list start:')
    ret=mergediff(merge_file)
    print ('Done: total modifies: %s' %(ret))
elif len(sys.argv) == 3: 
    print ('Only get diffs')       
    print ('Step1: compare two files: %s and %s' %(sys.argv[1], sys.argv[2]))
    [t1, t2]=parsefile(sys.argv[1],sys.argv[2])
    getdiff(t1, t2)
    print ('Step2: generate diffs in file %s, you can check it later' %(diff_file))
    #print ('Step3: merge diffs from %s, to %s_merge' %(merge_file, merge_file))
    #ret=mergediff(merge_file)
    print ('Done:')
else:
    print ('Usage: %s defconfig perf_defconfig [mergeconfig such as NX523J_defconfig]' %(sys.argv[0]))
