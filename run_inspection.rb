#!/usr/bin/ruby

require 'fileutils'

## This is a utility to run pinspect executable (not for calibration)

#Set this to point to correct dataset path
DATASET_DIR = "/home/matti/code/DatasetLoccioni/Stereo example 1/Stereo pairs"
PINSPECT_PKG_DIR = "."
PINSPECT_EXECUTABLE = "build/pinspect"
CALIBRATION_DIR = "results"



#Represents an object to run the inspection on
class DatasetEntry

  def initialize(name)
    check(name)
    @name = name
  end

  def compactName
    File.basename @name
  end

  def left
    File.join(@name, "Left.bmp")
  end

  def right
    File.join(@name, "Right.bmp")
  end

  def to_s
    "DatasetEntry:#{@name}"
  end

private
  def check(name)
    left =   File.join(name, "Left.bmp")
    if ! File.exist?(left)
      puts "#{name}/Left.bmp image not found"
      exit
    end

    right = File.join(name, "Right.bmp")
    if ! File.exist?(right)
      puts "#{name}/Right.bmp image not found"
      exit
    end
  end

end



#Read the dataset from disk
def readDataset
  entries = Array.new
  Dir.entries(DATASET_DIR).sort.each do |x|
    if x != "." and x != ".."
      entries << DatasetEntry.new(File.join(DATASET_DIR, x))
    end
  end
  entries
end



#Create the results directory needed by pinspect
def createResultsDir(datasetEntry)
  dirname = datasetEntry.compactName + "_results"
  Dir.mkdir(dirname) unless File.exist?(dirname)

  resultsDir = File.join(PINSPECT_PKG_DIR,  CALIBRATION_DIR)
  leftCamera = File.join(resultsDir, "leftCamera.xml")
  rightCamera = File.join(resultsDir, "rightCamera.xml")
  stereoCamera = File.join(resultsDir, "stereoCamera.xml")
  rectifyMaps = File.join(resultsDir, "rectifyMaps.xml")
  parametersFile = File.join(PINSPECT_PKG_DIR, "parameters.xml")
  
  #link to calibration parameters
  name = File.join(dirname, "leftCamera.xml")
  File.link(leftCamera, name) unless File.exist?(name)
  name = File.join(dirname, "rightCamera.xml")
  File.link(rightCamera, name) unless File.exist?(name)
  name = File.join(dirname, "stereoCamera.xml")
  File.link(stereoCamera, name) unless File.exist?(name)
  name = File.join(dirname, "rectifyMaps.xml")
  File.link(rectifyMaps, name) unless File.exist?(name)
  
  #copy parameters.xml
  name = File.join(dirname, "parameters.xml")
  FileUtils.cp(parametersFile, dirname) unless File.exist?(name)

  dirname
end



entries = readDataset
puts "Dataset entries found: \n\n"
entries.each_index {|x| puts entries[x].compactName + " [#{x}]" }

puts "\nChoose by index and pass commands to pinspect"

while true

print "\n>>"
commandLine = gets
commandLine = commandLine.split
index = commandLine[0].to_i
fromCmd = commandLine[1]
toCmd = commandLine[2]

if(fromCmd == nil)
  puts "need to specify command"
  next
end

if(toCmd == nil)
  toCmd = ""
end

#execute
choosed = entries[index]
resultsDir = createResultsDir choosed

execName = File.join(PINSPECT_PKG_DIR, PINSPECT_EXECUTABLE)
paramsOpt = "--parameters=" + File.join(resultsDir, "parameters.xml")
resultsOpt = "--results=" + resultsDir
leftImgOpt = "--left-image=" + choosed.left
rightImgOpt = "--right-image=" + choosed.right

#debug
puts "\nCall parameters to pinspect: "
puts execName, paramsOpt, resultsOpt, leftImgOpt, rightImgOpt, fromCmd, toCmd

puts "\nExecution of pinspect"
system(execName, paramsOpt, resultsOpt, leftImgOpt, rightImgOpt, fromCmd, toCmd)

end
