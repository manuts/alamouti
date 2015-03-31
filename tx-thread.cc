/*
 * Copyright (c) 2014 Manu T S
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <complex>
#include <pthread.h>
#include <getopt.h>
#include <liquid/liquid.h>
#include <liquid/alamouti.h>
#include <ctime>

#include <uhd/utils/thread_priority.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <gnuradio/filter/fft_filter.h>
#include <gnuradio/gr_complex.h>
#include <volk/volk.h>

#include <boost/thread/thread.hpp>
#include <boost/program_options.hpp>

#define USE_BOOST_THREAD true
#define SIMULATION false
#define SAVE_TO_FILE true
#define ONLINE_DECODING false
#define FIND_SAMPLE_OFFSET true
#define USE_X300 true

#define TX_DELAY 0
#define RX_DELAY 0

namespace po = boost::program_options;

static bool verbose;
unsigned long int rx_loop_number = 0;
unsigned long int tx_loop_number = 0;
time_t tx_begin, tx_end;
time_t rx_begin, rx_end;
size_t tx_buff_len;
size_t packer_buff_len;
size_t rx_buff_len;
size_t unpacker_buff_len;

typedef struct
{
  uhd::usrp::multi_usrp::sptr * tx;
  size_t tx_sig_len;
  std::complex<float> ** tx_sig;
  size_t * num_tx_samps;
  liquid::alamouti::framegen * packer;
} tx_thread_data;

void * tx_worker (void * _ptr)
{
  tx_thread_data * ptr = (tx_thread_data *)_ptr;
  std::vector<std::complex<float>*> tx_buff;
  std::complex<float> * packer_buff[2];
  std::vector<size_t> tx_chans;
  size_t num_samps_sent;
  size_t packer_buff_ptr;

  // create a tx streamer
  std::string cpu = "fc32";           // cpu format for the streamer
  std::string wire = "sc16";          // wire formate for the streamer
  for(size_t chan = 0; chan < 2; chan++)
    tx_chans.push_back(chan);
  uhd::stream_args_t tx_stream_args(cpu, wire);
  tx_stream_args.channels = tx_chans;
  uhd::tx_streamer::sptr tx_stream = (*(ptr->tx))->get_tx_stream(tx_stream_args);
  tx_buff_len = tx_stream->get_max_num_samps();
  tx_buff.resize(2);

  liquid::alamouti::delay dlay(TX_DELAY);

  uhd::tx_metadata_t txmd;
  (*(ptr->tx))->set_time_now(uhd::time_spec_t(0.0));
  txmd.start_of_burst = true;
  txmd.end_of_burst = false;
  txmd.has_time_spec = true;

  uhd::time_spec_t event(0.1);
  txmd.time_spec = event;
  tx_begin = time(NULL);

  // transmission
  {
    for (size_t chan = 0; chan < 2; chan++) { 
      packer_buff[chan] = (std::complex<float> *)malloc(packer_buff_len*sizeof(std::complex<float>));
    }

    assert(packer_buff_len > tx_buff_len);

    while(true)
    {
      if(*(ptr->num_tx_samps) + packer_buff_len > ptr->tx_sig_len)
        break;
      tx_loop_number++;

      assert(packer_buff_len == (ptr->packer)->work(packer_buff));
      packer_buff_ptr = 0;
      
      dlay.work(packer_buff[0], packer_buff_len);

      while(packer_buff_ptr + tx_buff_len < packer_buff_len) {
        tx_buff[0] = packer_buff[0] + packer_buff_ptr;
        tx_buff[1] = packer_buff[1] + packer_buff_ptr;
        num_samps_sent = tx_stream->send(tx_buff,
                                         tx_buff_len,
                                         txmd);
        std::cout << num_samps_sent;
        packer_buff_ptr += num_samps_sent;
      }
      tx_buff[0] = packer_buff[0] + packer_buff_ptr;
      tx_buff[1] = packer_buff[1] + packer_buff_ptr;
      assert(packer_buff_len - packer_buff_ptr == 
             tx_stream->send(tx_buff,
                             packer_buff_len - packer_buff_ptr,
                             txmd));

      if(SAVE_TO_FILE) {
        for(size_t chan = 0; chan < 2; chan++)
          memmove((ptr->tx_sig)[chan] + *(ptr->num_tx_samps),
              packer_buff[chan], packer_buff_len*sizeof(std::complex<float>));
      }

      *(ptr->num_tx_samps) += packer_buff_len;
      txmd.start_of_burst = false;
      txmd.has_time_spec = false;
    }
  }
  // transmission ends

  tx_end = time(NULL);
  std::cout << "Exiting tx thread\n";
  // send the last packet
  txmd.end_of_burst = true;
  tx_stream->send(tx_buff,
                  0,
                  txmd);

  if(USE_BOOST_THREAD)
    return NULL;
  else
    pthread_exit(NULL);
}

int UHD_SAFE_MAIN(int argc, char **argv)
{
  uhd::set_thread_priority_safe();

  // operating parameters
  double cent_freq;         // center frequency of transmission
  double samp_rate;         // usrp samping rate
  float tone_amp;           // tone amplitude
  double txgain;            // tx frontend gain
  double num_secs;          // number of seconds to operate
  int delay;

  // default values
  double d_cent_freq;
  double d_samp_rate;
  if(USE_X300) {
    d_cent_freq = 2600.0e6;
    d_samp_rate = 1000e3;
  }
  else {
    d_cent_freq = 1700.0e6;
    d_samp_rate = 500e3;
  }
  float d_tone_amp   = 0.25;
  double d_txgain    = 15.0;
  double d_num_secs  = 10.0;
  bool d_verbose     = true;

  //set the operating parameters
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "help message")
    ("freq", po::value<double>(&cent_freq)->default_value(d_cent_freq), "RF center frequency in Hz")
    ("rate", po::value<double>(&samp_rate)->default_value(d_samp_rate), "USRP Sampling rate")
    ("rrc_amplitude", po::value<float>(&tone_amp)->default_value(d_tone_amp), "rrc pulse shape amplitude")
    ("tx_gain", po::value<double>(&txgain)->default_value(d_txgain), "TX Front end gain")
    ("duration", po::value<double>(&num_secs)->default_value(d_num_secs), "Number of seconds to run")
    ("verbose", po::value<bool>(&verbose)->default_value(d_verbose), "Verbose")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  //print the help message
  if (vm.count("help")) {
    std::cout << boost::format("mimo-test %s") % desc << std::endl;
    std::cout
      << std::endl
      << "Testing MIMO.\n"
      << std::endl;
      return 0;
  }

  // pulse shape properties
  unsigned int k  = 4;                // samples/symbol
  unsigned int m  = 3;                // filter delay (symbols)
  float beta      = 0.50f;            // excess bandwidth factor

  // channel properties
  size_t num_tx_chans;

  // rest
  size_t tx_sig_len;
  size_t num_tx_samps;

  tx_thread_data * tx_data;

  std::complex<float> * tx_sig[2];
  boost::thread_group txrx_thread;
  pthread_t tx_thread;

  liquid::alamouti::framegen packer(k, m, beta);
  packer_buff_len = packer.get_frame_len();

  num_tx_samps = 0;
  tx_sig_len = (size_t)(num_secs*samp_rate);

  {
    uhd::device_addr_t tx_addr;
    uhd::usrp::multi_usrp::sptr tx;

    if(USE_X300) {
      tx_addr["addr0"] = "192.168.10.2";
      tx = uhd::usrp::multi_usrp::make(tx_addr);
      uhd::usrp::subdev_spec_t tx_subdev_spec("A:0 B:0");
      tx->set_tx_subdev_spec(tx_subdev_spec, uhd::usrp::multi_usrp::ALL_MBOARDS);
    }

    else {
      tx_addr["addr0"] = "134.147.118.215";
      tx_addr["addr1"] = "134.147.118.210";
      tx = uhd::usrp::multi_usrp::make(tx_addr);
      tx->set_clock_source("mimo", 1);
      tx->set_time_source("mimo", 1);
    }
  
    num_tx_chans = tx->get_tx_num_channels();
    for (size_t chan = 0; chan < num_tx_chans; chan++) {
      tx->set_tx_rate(samp_rate, chan);
      uhd::tune_request_t tx_tune_request(cent_freq);
      uhd::tune_result_t tx_tune_result;
      tx_tune_result = tx->set_tx_freq(tx_tune_request, chan);
      tx->set_tx_gain(txgain, chan);
      tx->set_tx_antenna("TX/RX", chan);
    }

    // allocate memory to store the transmit signal
    if(SAVE_TO_FILE) {
      for (size_t chan = 0; chan < num_tx_chans; chan++) {
        tx_sig[chan] = (std::complex<float> *)malloc(tx_sig_len*sizeof(std::complex<float>));
      }
    }

    printf("constructing the thread input data structures\n");
    tx_data = (tx_thread_data *)malloc(sizeof(tx_thread_data));
    tx_data->tx = &tx;
    tx_data->tx_sig_len = tx_sig_len;
    tx_data->tx_sig = tx_sig;
    tx_data->num_tx_samps = &(num_tx_samps);
    tx_data->packer = &packer;

    std::cout << "Invoking worker threads\n";
    if(USE_BOOST_THREAD) {
      std::cout << "Invoking boost threads\n";
      txrx_thread.create_thread(boost::bind(tx_worker, (void *)tx_data));
      txrx_thread.join_all();
    }
    else {
      std::cout << "Invoking pthread thread\n";
      if(pthread_create(&tx_thread, NULL, tx_worker, (void *)tx_data)){
        std::cout << "Error invoking tx thread\n";
        return 1;
      }
      pthread_join(tx_thread, NULL);
    }
  
    if(SAVE_TO_FILE){
      FILE * f_tx_sig1;
      FILE * f_tx_sig2;
      f_tx_sig1 = fopen("/tmp/tx_sig1", "wb");
      f_tx_sig2 = fopen("/tmp/tx_sig2", "wb");
      fwrite((void *)(tx_sig[0]), sizeof(std::complex<float>), num_tx_samps, f_tx_sig1);
      fwrite((void *)(tx_sig[1]), sizeof(std::complex<float>), num_tx_samps, f_tx_sig2);
      fclose(f_tx_sig1);
      fclose(f_tx_sig2);
      for (size_t chan = 0; chan < num_tx_chans; chan++) {
        free(tx_sig[chan]);
      }
    }
  }
  printf("Delay : %d\n", delay);
  std::cout << "Frame Length = " << packer_buff_len << std::endl;
  std::cout << "num_tx_samps = " << num_tx_samps << std::endl;
  std::cout << "TX Loop Number = " << tx_loop_number << std::endl;
  std::cout << "TX Runtime = " << tx_begin - tx_end << std::endl;
  return EXIT_SUCCESS;
}
