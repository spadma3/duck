# Imports
from helpers.DummyByteMultiArray import DummyMultiArrayDimension as dmad
from helpers.DummyByteMultiArray import DummyMultiArrayLayout as dmal
from helpers.DummyByteMultiArray import DummyByteMultiArray as dbma
from libserialize import serialize
from libserialize import parse

dmad_1 = dmad("label1", 15, 20)
dmad_2 = dmad("label2", 25, 30)
dmad_3 = dmad("label3", 35, 40)

dmal_1 = dmal(5)
dmal_1.add(dmad_1)
dmal_1.add(dmad_2)
dmal_1.add(dmad_3)

i = 0
for element in dmal_1.dim:
    print "{}: label: {}, size: {}, stride: {}".format(i, element.label, element.size, element.stride)
    i += 1

dbma_1 = dbma(dmal_1, "test_data")

data_serialized = serialize("data_name", dbma_1)

dmad_4 = dmad("label4", 500, 600)
dmal_2 = dmal(1000)
dbma_2 = dbma(dmal_2, "receiving_data")

name, dbma_parsed = parse(data_serialized, dmad_4, dbma_2)

print "name: {}".format(name)
data = dbma_parsed.data
print "data: {}".format(data)
print "layout:"
layout = dbma_parsed.layout
print "data_offset: {}".format(layout.data_offset)
print "multi array dimension:"
dim = layout.dim
i = 0
for element in dim:
    print "{}: label: {}, size: {}, stride: {}".format(i, element.label, element.size, element.stride)
    i += 1