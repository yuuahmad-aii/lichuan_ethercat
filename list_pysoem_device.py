import pysoem
for d in pysoem.find_adapters():
    print(d.name, d.desc)