package cs.ucla.edu.bwaspark

import org.apache.spark.SparkContext
import org.apache.spark.SparkContext._
import org.apache.spark.SparkConf
import org.apache.spark.rdd.RDD

import scala.collection.mutable.MutableList

import cs.ucla.edu.bwaspark.datatype._
import cs.ucla.edu.bwaspark.worker1.BWAMemWorker1Batched._
import cs.ucla.edu.bwaspark.worker2.BWAMemWorker2._
import cs.ucla.edu.bwaspark.debug.DebugFlag._
import cs.ucla.edu.bwaspark.fastq._
import cs.ucla.edu.avro.fastq._

import java.io.FileReader
import java.io.BufferedReader

import accUCLA.api._

// Profiling
import cs.ucla.edu.bwaspark.profiling.SWBatchTimeBreakdown

object BWAMEMSpark {
  // load reads from the FASTQ file (for testing use)
  private def loadFASTQSeqs(fileName: String, readNum: Int): Array[String] = {
    
    val reader = new BufferedReader(new FileReader(fileName))
    var line = reader.readLine
    var i = 0
    var readIdx = 0    
    var seqs = new Array[String](readNum / 4)

    while(line != null) {
      if(i % 4 == 1) {
        seqs(readIdx) = line
        readIdx += 1
      }
      i += 1
      line = reader.readLine
    } 

    //seqs.foreach(println(_))
    seqs
  }

  class testRead {
    var seq: String = _
    var regs: MutableList[MemAlnRegType] = new MutableList[MemAlnRegType]
  }

  def main(args: Array[String]) {
    //val sc = new SparkContext("local[96]", "BWA-mem Spark",
       //"/home/hadoopmaster/spark/spark-0.9.0-incubating-bin-hadoop2-prebuilt/", List("/home/ytchen/incubator/bwa-spark-0.3.1/target/bwa-spark-0.3.1.jar"))
    val conf = new SparkConf().setAppName("BWA-mem Spark").set("spark.akka.frameSize", "128").set("spark.logConf", "true")
    val sc = new SparkContext(conf)

    //val fastqLoader = new FASTQLocalFileLoader(10000000)
    //val fastqLoader = new FASTQLocalFileLoader(40000000)
    //val fastqLoader = new FASTQLocalFileLoader(200000000)
    //fastqLoader.storeFASTQInHDFS(sc, "/home/ytchen/genomics/data/HCC1954_1_10Mreads.fq", "hdfs://Jc11:9000/user/ytchen/data/HCC1954_1_10Mreads")
    //fastqLoader.storeFASTQInHDFS(sc, "/home/pengwei/HCC1954_1_20reads.fq", "hdfs://Jc11:9000/user/pengwei/data/HCC1954_1_20reads")
    //fastqLoader.storeFASTQInHDFS(sc, "/home/ytchen/genomics/data/HCC1954_1_100reads.fq", "hdfs://Jc11:9000/user/ytchen/data/HCC1954_1_100reads")
    //fastqLoader.storeFASTQInHDFS(sc, "/home/ytchen/genomics/data/ERR013140_1.filt.fastq", "hdfs://Jc11:9000/user/ytchen/data/ERR013140_1.filt.fastq_96")
    //fastqLoader.storeFASTQInHDFS(sc, "/home/pengwei/genomics/InputFiles/HCC1954_1.fq", "hdfs://Jc11:9000/user/ytchen/data/HCC1954_1.fq")
    //fastqLoader.storeFASTQInHDFS(sc, "/cdsc_nfs/cdsc0/software/spark/genomics_data/ManmadeFASTQ_1.fq", "hdfs://cdsc0:9000/user/pengwei/data/ManmadeFASTQ_1.fq")

    //loading index files
    println("Load Index Files")
    val bwaIdx = new BWAIdxType
    val prefix = "/cdsc_nfs/cdsc0/software/spark/genomics_data/ReferenceMetadata/human_g1k_v37.fasta"
    bwaIdx.load(prefix, 0)

    //loading BWA MEM options
    println("Load BWA-MEM options")
    val bwaMemOpt = new MemOptType
    bwaMemOpt.load

    //val bwaIdxBWTGlobal = sc.broadcast(bwaIdx.bwt)
    //val bwaIdxBNSGlobal = sc.broadcast(bwaIdx.bns)
    //val bwaIdxPACGlobal = sc.broadcast(bwaIdx.pac)
    val bwaIdxGlobal = sc.broadcast(bwaIdx)
    //val bwaIdxGlobal = sc.broadcast(bwaIdx, prefix)
    val bwaMemOptGlobal = sc.broadcast(bwaMemOpt)

    //debugLevel = 1

    //val fastqRDDLoader = new FASTQRDDLoader(sc, "hdfs://Jc11:9000/user/pengwei/data/HCC1954_1_20reads", 1)
    //val fastqRDDLoader = new FASTQRDDLoader(sc, "hdfs://Jc11:9000/user/ytchen/data/HCC1954_1_20reads", 1)
    //val fastqRDDLoader = new FASTQRDDLoader(sc, "hdfs://Jc11:9000/user/ytchen/data/HCC1954_1_100reads", 1)
    //val fastqRDDLoader = new FASTQRDDLoader(sc, "hdfs://Jc11:9000/user/ytchen/data/HCC1954_1_10Mreads", 2)
    //val fastqRDDLoader = new FASTQRDDLoader(sc, "hdfs://Jc11:9000/user/pengwei/data/ManmadeFASTQ.fq", 41)
    //val fastqRDDLoader = new FASTQRDDLoader(sc, "hdfs://cdsc0:9000/user/pengwei/data/ManmadeFASTQ_1.fq", 41)
    val fastqRDDLoader = new FASTQRDDLoader(sc, "hdfs://cdsc0:9000/user/pengwei/data/ManmadeFASTQ_1.fq", 39)
    //val fastqRDDLoader = new FASTQRDDLoader(sc, "hdfs://Jc11:9000/user/pengwei/data/HCC1954_1.fq", 201)
    //val fastqRDD = fastqRDDLoader.RDDLoad("hdfs://Jc11:9000/user/ytchen/data/ERR013140_1.filt.fastq_new/0")
    val fastqRDD = fastqRDDLoader.RDDLoadAll
    //println("fastqRDD.count = " + fastqRDD.count)
    //fastqRDD.cache()
    //if (useFPGA == true) conn = new Connector2FPGA("127.0.0.1", 5000) 
    //conn.buildConnection( 1 );

    //def it2ArrayIt(iter: Iterator[FASTQRecord]): Iterator[Array[ReadType]] = {
    // *****     PROFILING    *****
    def it2ArrayIt(iter: Iterator[FASTQRecord]): Iterator[SWBatchTimeBreakdown] = {
      val batchedDegree = 1024
      var counter = 0

      //var ret = new MutableList[Array[ReadType]]
      // *****     PROFILING    *****
      var ret = new MutableList[SWBatchTimeBreakdown]

      //var node = new MutableList[FASTQRecord]
      var node = new Array[FASTQRecord](batchedDegree)
      while (iter.hasNext) {
              node(counter) = iter.next
              counter = counter + 1
              if (counter == batchedDegree) {
          //println("[ERROR]Should not be executed")
                ret += bwaMemWorker1Batched(bwaMemOptGlobal.value, bwaIdxGlobal.value.bwt, bwaIdxGlobal.value.bns, bwaIdxGlobal.value.pac, null, node, batchedDegree)
                counter = 0
              }
      }
      //println("counter = " + counter)
      if (counter != 0)
      	ret += bwaMemWorker1Batched(bwaMemOptGlobal.value, bwaIdxGlobal.value.bwt, bwaIdxGlobal.value.bns, bwaIdxGlobal.value.pac, null, node, counter)
      ret.toArray.iterator
    }

    //val readsArray = fastqRDD.mapPartitions(it2ArrayIt)
    //println("[DEBUG]readsArray.count = " + readsArray.count)

    // *****     PROFILING    *****
    val profilingData = fastqRDD.mapPartitions(it2ArrayIt).collect

    //if (useFPGA) 
    //  conn.closeConnection();
    
    //val reads = fastqRDD.map( seq => bwaMemWorker1(bwaMemOptGlobal.value, bwaIdxGlobal.value.bwt, bwaIdxGlobal.value.bns, bwaIdxGlobal.value.pac, null, seq) )
    //val readsArray = batchedRDD.map( seqs => bwaMemWorker1Batched(bwaMemOptGlobal.value, bwaIdxGlobal.value.bwt, bwaIdxGlobal.value.bns, bwaIdxGlobal.value.pac, null, seqs, seqs.length) )
    //val reads = fastqRDD.map( seq => bwaMemWorker1(bwaMemOptGlobal.value, bwaIdxBWTGlobal.value, bwaIdxBNSGlobal.value, bwaIdxPACGlobal.value, null, seq) )
    //val c = reads.count
    //println("Count: " + c)
    //println("Num of Partitions: " + readsArray.count)
    //readsArray.foreach( ele => { println("Num of Reads: " + ele.length) } )
    //val numOfReads = readsArray.map(ele => ele.length).reduce((a, b) => a+b);
    //println("[DEBUG]NumOfReads = " + numOfReads);
    //println("Count: " + reads.map( read => bwaMemWorker2(bwaMemOptGlobal.value, read.regs, bwaIdxGlobal.value.bns, bwaIdxGlobal.value.pac, read.seq, 0) ).reduce(_ + _))
    
  } 
}
