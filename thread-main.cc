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

#define TX_DELAY 12
#define RX_DELAY 12

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
  size_t tx_sig_len;
  size_t rx_sig_len;
  size_t * num_tx_samps;
  size_t * num_rx_samps;
  liquid::alamouti::framegen * packer;
  liquid::alamouti::framesync * unpacker;
} simulator_data;

void * simulator (void * _ptr)
{
  simulator_data * ptr = (simulator_data *)_ptr;
  unsigned int buff_len = (ptr->packer)->get_frame_len();
  std::cout << buff_len << std::endl;
  std::complex<float> * buff[2];
  std::complex<float> attenuation = 1.0f;
  for (size_t chan = 0; chan < 2; chan++) 
    buff[chan] = (std::complex<float> *)malloc(buff_len*sizeof(std::complex<float>));
  while(true)
  {
    if(*(ptr->num_rx_samps) + buff_len > ptr->rx_sig_len)
      break;
    assert(buff_len == (ptr->packer)->work(buff));
    for (unsigned int i = 0; i < buff_len; i++) {
      buff[0][i] = buff[0][i]*attenuation;
      buff[1][i] = buff[1][i]*attenuation;
    }
    (ptr->unpacker)->work(buff, buff_len);
    if(*(ptr->num_tx_samps) + buff_len > ptr->tx_sig_len)
      break;
    *(ptr->num_tx_samps) += buff_len;
    *(ptr->num_rx_samps) += buff_len;
  }
  std::cout << "Exiting Simulator\n";
  return NULL;
}

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
  unsigned int i;

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
  txmd.start_of_burst = true;
  txmd.end_of_burst = false;

  (*(ptr->tx))->set_time_now(uhd::time_spec_t(0.0), 0);
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
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
      
      dlay.work(packer_buff[0], packer_buff_len);

      for(i = 0; i < packer_buff_len/tx_buff_len; i++) {
        tx_buff[0] = packer_buff[0] + i*tx_buff_len;
        tx_buff[1] = packer_buff[1] + i*tx_buff_len;
        assert(tx_buff_len == tx_stream->send(tx_buff,
                                              tx_buff_len,
                                              txmd));
      }
      tx_buff[0] = packer_buff[0] + i*tx_buff_len;
      tx_buff[1] = packer_buff[1] + i*tx_buff_len;
      assert(packer_buff_len - i*tx_buff_len == tx_stream->send(tx_buff,
                                                                packer_buff_len - i*tx_buff_len,
                                                                txmd));

      if(SAVE_TO_FILE) {
        for(size_t chan = 0; chan < 2; chan++)
          memmove((ptr->tx_sig)[chan] + *(ptr->num_tx_samps),
              packer_buff[chan], packer_buff_len*sizeof(std::complex<float>));
      }

      *(ptr->num_tx_samps) += packer_buff_len;
      txmd.start_of_burst = false;
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

typedef struct{
  uhd::usrp::multi_usrp::sptr * rx;
  size_t rx_sig_len;
  std::complex<float> ** rx_sig;
  size_t * num_rx_samps;
  liquid::alamouti::framesync * unpacker;
} rx_thread_data;

void * rx_worker (void * _ptr)
{
  rx_thread_data * ptr = (rx_thread_data *)_ptr;
  std::vector<std::complex<float>*> rx_buff;
  std::complex<float> * unpacker_buff[2];
  std::vector<size_t> rx_chans;
  float timeout = 1.2;
  unsigned int i;

  // create a rx streamer
  std::string cpu = "fc32";           // cpu format for the streamer
  std::string wire = "sc16";          // wire formate for the streamer
  for(size_t chan = 0; chan < 2; chan++)
    rx_chans.push_back(chan);
  uhd::stream_args_t rx_stream_args(cpu, wire);
  rx_stream_args.channels = rx_chans;
  uhd::rx_streamer::sptr rx_stream = (*(ptr->rx))->get_rx_stream(rx_stream_args);
  rx_buff_len = rx_stream->get_max_num_samps();
  rx_buff.resize(2);

  liquid::alamouti::delay dlay(RX_DELAY);

  uhd::rx_metadata_t rxmd;

  (*(ptr->rx))->set_time_now(uhd::time_spec_t(0.0), 0);
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
  stream_cmd.stream_now = false;
  stream_cmd.time_spec = uhd::time_spec_t(0.5);
  rx_stream->issue_stream_cmd(stream_cmd);
  rx_begin = time(NULL);

  // reception
  {
    for (size_t chan = 0; chan < 2; chan++) {
      unpacker_buff[chan] = (std::complex<float> *)malloc(unpacker_buff_len*sizeof(std::complex<float>));
    }
    
    assert(unpacker_buff_len > rx_buff_len);

    while(true)
    {
      if(*(ptr->num_rx_samps) + unpacker_buff_len > ptr->rx_sig_len)
        break;
      rx_loop_number++;
      
      for(i = 0; i < unpacker_buff_len/rx_buff_len; i++) {
        rx_buff[0] = unpacker_buff[0] + i*rx_buff_len;
        rx_buff[1] = unpacker_buff[1] + i*rx_buff_len;
        assert(rx_buff_len == rx_stream->recv(rx_buff,
                                              rx_buff_len,
                                              rxmd,
                                              timeout));
        if(rxmd.error_code) {
          std::cerr << rxmd.strerror() << "\n";
          break;
        }
      } 
      if(rxmd.error_code) {
        std::cerr << rxmd.strerror() << "\n";
        break;
      }
      rx_buff[0] = unpacker_buff[0] + i*rx_buff_len;
      rx_buff[1] = unpacker_buff[1] + i*rx_buff_len;
      assert(unpacker_buff_len - i*rx_buff_len == rx_stream->recv(rx_buff,
                                                                  unpacker_buff_len - i*rx_buff_len,
                                                                  rxmd,
                                                                  timeout));
      if(rxmd.error_code) {
        std::cerr << rxmd.strerror() << "\n";
        break;
      }

      dlay.work(unpacker_buff[1], unpacker_buff_len);

      if(ONLINE_DECODING)
        (ptr->unpacker)->work(unpacker_buff, unpacker_buff_len);
      
      if(SAVE_TO_FILE) {
        for(size_t chan = 0; chan < 2; chan++)
          memmove((ptr->rx_sig)[chan] + *(ptr->num_rx_samps),
              unpacker_buff[chan], unpacker_buff_len*sizeof(std::complex<float>));
      }

      *(ptr->num_rx_samps) += unpacker_buff_len;
      
      timeout = 0.1;
    }
  }
  // reception ends

  rx_end = time(NULL);
  std::cout << "Exiting rx thread\n";
  // stop rx streaming
  stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
  rx_stream->issue_stream_cmd(stream_cmd);

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
  double rxgain;            // rx frontend gain
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
  double d_rxgain    = 15.0;
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
    ("rx_gain", po::value<double>(&rxgain)->default_value(d_rxgain), "RX Front end gain")
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
  size_t num_rx_chans;
  size_t num_tx_chans;
  std::vector<size_t> rx_chans;

  // rest
  size_t tx_sig_len;
  size_t rx_sig_len;
  size_t num_rx_samps, num_tx_samps;

  tx_thread_data * tx_data;
  rx_thread_data * rx_data;
  simulator_data * sm_data;

  std::complex<float> * tx_sig[2];
  std::complex<float> * rx_sig[2];
  boost::thread_group txrx_thread;
  pthread_t tx_thread, rx_thread;

  liquid::alamouti::framegen packer(k, m, beta);
  liquid::alamouti::framesync fs(k, m, beta);
  packer_buff_len = packer.get_frame_len();
  unpacker_buff_len = fs.get_frame_len(); 
  assert(packer_buff_len == unpacker_buff_len);

  num_tx_samps = 0;
  num_rx_samps = 0;
  tx_sig_len = (size_t)(num_secs*samp_rate);
  rx_sig_len = (size_t)(num_secs*samp_rate);

  if(SIMULATION) {
    sm_data = (simulator_data *)malloc(sizeof(simulator_data));
    sm_data->tx_sig_len = tx_sig_len;
    sm_data->rx_sig_len = rx_sig_len;
    sm_data->num_tx_samps = &(num_tx_samps);
    sm_data->num_rx_samps = &(num_rx_samps);
    sm_data->packer = &packer;
    sm_data->unpacker = &fs;

    simulator((void *)sm_data);
  }
  else {
    uhd::device_addr_t tx_addr, rx_addr;
    uhd::usrp::multi_usrp::sptr tx;
    uhd::usrp::multi_usrp::sptr rx;

    if(USE_X300) {
      tx_addr["addr0"] = "134.147.118.216";
      rx_addr["addr0"] = "134.147.118.217";
      tx = uhd::usrp::multi_usrp::make(tx_addr);
      rx = uhd::usrp::multi_usrp::make(rx_addr);
      uhd::usrp::subdev_spec_t tx_subdev_spec("A:0 B:0");
      uhd::usrp::subdev_spec_t rx_subdev_spec("A:0 B:0");
      tx->set_tx_subdev_spec(tx_subdev_spec, uhd::usrp::multi_usrp::ALL_MBOARDS);
      rx->set_rx_subdev_spec(rx_subdev_spec, uhd::usrp::multi_usrp::ALL_MBOARDS);
    }

    else {
      rx_addr["addr0"] = "134.147.118.214";
      rx_addr["addr1"] = "134.147.118.212";
      tx_addr["addr0"] = "134.147.118.215";
      tx_addr["addr1"] = "134.147.118.210";
      tx = uhd::usrp::multi_usrp::make(tx_addr);
      rx = uhd::usrp::multi_usrp::make(rx_addr);

      tx->set_clock_source("mimo", 1);
      tx->set_time_source("mimo", 1);
      rx->set_clock_source("mimo", 1);
      rx->set_time_source("mimo", 1);
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
    num_rx_chans = rx->get_rx_num_channels();
    for (size_t chan = 0; chan < num_rx_chans; chan++) {
      rx->set_rx_rate(samp_rate, chan);
      uhd::tune_request_t rx_tune_request(cent_freq);
      uhd::tune_result_t rx_tune_result;
      rx_tune_result = rx->set_rx_freq(rx_tune_request, chan);
      rx->set_rx_gain(rxgain, chan);
      rx->set_rx_antenna("TX/RX", chan);
    }

    // allocate memory to store the transmit signal
    if(SAVE_TO_FILE) {
      for (size_t chan = 0; chan < num_tx_chans; chan++) {
        tx_sig[chan] = (std::complex<float> *)malloc(tx_sig_len*sizeof(std::complex<float>));
      }
    }
    // allocate memory to store the received samples
    if(SAVE_TO_FILE) {
      for (size_t chan = 0; chan < num_rx_chans; chan++) {
        rx_sig[chan] = (std::complex<float> *)malloc(rx_sig_len*sizeof(std::complex<float>));
      }
    }

    printf("constructing the thread input data structures\n");
    tx_data = (tx_thread_data *)malloc(sizeof(tx_thread_data));
    rx_data = (rx_thread_data *)malloc(sizeof(rx_thread_data));
    tx_data->tx = &tx;
    rx_data->rx = &rx;
    tx_data->tx_sig_len = tx_sig_len;
    rx_data->rx_sig_len = rx_sig_len;
    tx_data->tx_sig = tx_sig;
    rx_data->rx_sig = rx_sig;
    tx_data->num_tx_samps = &(num_tx_samps);
    rx_data->num_rx_samps = &(num_rx_samps);
    tx_data->packer = &packer;
    rx_data->unpacker = &fs;

    std::cout << "Invoking worker threads\n";
    if(USE_BOOST_THREAD) {
      std::cout << "Invoking worker threads\n";
      txrx_thread.create_thread(boost::bind(tx_worker, (void *)tx_data));
      txrx_thread.create_thread(boost::bind(rx_worker, (void *)rx_data));
      txrx_thread.join_all();
    }
    else {
      if(pthread_create(&tx_thread, NULL, tx_worker, (void *)tx_data)){
        std::cout << "Error invoking tx thread\n";
        return 1;
      }
      if(pthread_create(&rx_thread, NULL, rx_worker, (void *)rx_data)){
        std::cout << "Error invoking rx thread\n";
        return 1;
      }
      pthread_join(tx_thread, NULL);
      pthread_join(rx_thread, NULL);
    }

    if(FIND_SAMPLE_OFFSET)
    {
      float max = 0.0;
      float mag;
      std::complex<float> xcorr;
      int i = num_rx_samps/2;
      for(int j = i - 20; j < i + 20; j++)
      {
        volk_32fc_x2_dot_prod_32fc(&xcorr, rx_sig[0] + i, rx_sig[1] + j, 1024);
        mag = std::abs(xcorr);
        printf("xcorr[%3d] :%8.4f\n", j - i, mag);
        if(mag > max) {
          max = mag;
          delay = j - i;
        }
      }
    }

    // offline decoding
    if(!ONLINE_DECODING)
    {
      if(SAVE_TO_FILE)
      {
        std::complex<float> * unpacker_buff[2];
        for(size_t loop = 0; loop < rx_loop_number; loop++)
        {
          unpacker_buff[0] = rx_sig[0] + loop*unpacker_buff_len;
          unpacker_buff[1] = rx_sig[1] + loop*unpacker_buff_len;
          fs.work(unpacker_buff, unpacker_buff_len);
        }
      }
      else
      {
        std::cout << "No data saved to be decoded. Please set the SAVE_TO_FILE flag\n";
        exit(1);
      }
    }
  
    if(SAVE_TO_FILE){
      FILE * f_rx_sig1;
      FILE * f_rx_sig2;
      FILE * f_tx_sig1;
      FILE * f_tx_sig2;
      f_rx_sig1 = fopen("/tmp/rx_sig1", "wb");
      f_rx_sig2 = fopen("/tmp/rx_sig2", "wb");
      f_tx_sig1 = fopen("/tmp/tx_sig1", "wb");
      f_tx_sig2 = fopen("/tmp/tx_sig2", "wb");
      fwrite((void *)(rx_sig[0]), sizeof(std::complex<float>), num_rx_samps, f_rx_sig1);
      fwrite((void *)(rx_sig[1]), sizeof(std::complex<float>), num_rx_samps, f_rx_sig2);
      fwrite((void *)(tx_sig[0]), sizeof(std::complex<float>), num_tx_samps, f_tx_sig1);
      fwrite((void *)(tx_sig[1]), sizeof(std::complex<float>), num_tx_samps, f_tx_sig2);
      fclose(f_rx_sig1);
      fclose(f_rx_sig2);
      fclose(f_tx_sig1);
      fclose(f_tx_sig2);
      for (size_t chan = 0; chan < num_tx_chans; chan++) {
        free(tx_sig[chan]);
      }
      for (size_t chan = 0; chan < num_rx_chans; chan++) {
        free(rx_sig[chan]);
      }
    }
  }
  printf("Delay : %d\n", delay);
  std::cout << "Frame Length = " << tx_buff_len << std::endl;
  std::cout << "num_tx_samps = " << num_tx_samps << std::endl;
  std::cout << "num_rx_samps = " << num_rx_samps << std::endl;
  std::cout << "TX Loop Number = " << tx_loop_number << std::endl;
  std::cout << "RX Loop Number = " << rx_loop_number << std::endl;
  std::cout << "TX Runtime = " << tx_begin - tx_end << std::endl;
  std::cout << "RX Runtime = " << rx_begin - rx_end << std::endl;
  std::cout << "Bit Error Rate 1 = " << (double)fs.get_num_errors1()/(double)fs.get_num_bits_detected() << std::endl;
  std::cout << "Bit Error Rate 2 = " << (double)fs.get_num_errors2()/(double)fs.get_num_bits_detected() << std::endl;
  return EXIT_SUCCESS;
}
