#include <rr_platform/EthernetSocket.h>
#include <iostream>
#include <chrono>

using boost::asio::ip::tcp;



rr::EthernetSocket::EthernetSocket(std::string ip_addr, int port)
{
  // resolve all possible endpoints
  tcp::resolver resolver(io_service_);
  tcp::resolver::query query(ip_addr, std::to_string(port));
  tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

  // look through endpoints and hit socket's connect() member function until
  // a successful TCP connection is established
  this->sock_ = std::make_unique<tcp::socket>(io_service_);
  boost::asio::connect(*sock_, endpoint_iterator);
}

rr::EthernetSocket::~EthernetSocket()
{
  // shut down the TCP connection
  this->sock_->shutdown(boost::asio::ip::tcp::socket::shutdown_send);
}

void rr::EthernetSocket::send(const std::string& msg)
{
  boost::system::error_code error;
  boost::asio::write(*sock_, boost::asio::buffer(msg), error);
}

// https://stackoverflow.com/questions/40550730/how-to-implement-timeout-for-function-in-c
// Kinda hacky... But wanted timeout for read if disconnected 
template <typename TF, typename TDuration, class... TArgs>
std::string run_with_timeout(TF&& f, TDuration timeout, TArgs&&... args)
{
    using R = std::result_of_t<TF&&(TArgs&&...)>;
    std::packaged_task<R(TArgs...)> task(f);
    auto future = task.get_future();
    std::thread thr(std::move(task), std::forward<TArgs>(args)...);
    if (future.wait_for(timeout) != std::future_status::timeout)
    {
       thr.join();
       return future.get(); 
    }
    else
    {
       thr.detach(); 
       return "TIME_OUT";
    }
}

std::string rr::EthernetSocket::read() {
       boost::asio::streambuf buf;
       boost::system::error_code error;
       boost::asio::read_until(*sock_, buf, ";", error);
       std::string data = boost::asio::buffer_cast<const char*>(buf.data());
       return data;
}

std::string rr::EthernetSocket::read_with_timeout() {
  return run_with_timeout(std::bind(&rr::EthernetSocket::read, this), (std::chrono::seconds)1);
}

std::string rr::EthernetSocket::getIP()
{
  return sock_->remote_endpoint().address().to_string();
}

int rr::EthernetSocket::getPort()
{
  return static_cast<int>(sock_->remote_endpoint().port());
}

std::string rr::EthernetSocket::getBoostVersion()
{
  std::stringstream version;
  version << BOOST_VERSION / 100000 << "."      // major version
          << BOOST_VERSION / 100 % 1000 << "."  // minor version
          << BOOST_VERSION % 100;               // patch level

  return version.str();
}

