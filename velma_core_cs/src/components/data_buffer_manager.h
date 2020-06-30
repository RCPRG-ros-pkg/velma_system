// Copyright (c) 2019, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Authors: Szymon Jarocki, Dawid Seredyński
//

#include <rtt/RTT.hpp>
#include <rtt/Logger.hpp>

using namespace RTT;

template<typename data_type_> 
class DataBufferManagerSupport
{
public:
	DataBufferManagerSupport(RTT::TaskContext& tc, const std::string& port_name)
			: port_(port_name)
			, history_idx_(0)
			, max_history_length_(1000*60*60*10) // 1000 Hz * 60*60*10 s = 36 000 000 // maximum duration: 10 hours
			, history_(max_history_length_)
			, history_valid_(max_history_length_)
			, buffer_full_(false)
	{
		tc.ports()->addPort(port_);
	}

	void port_read()
	{
		data_type_ data;

		if (port_.read(data) == RTT::NewData)
		{
			history_[history_idx_] = data;
			history_valid_[history_idx_] = true;
		}
		else 
		{
			history_valid_[history_idx_] = false;  	
		}

		++history_idx_;

		if (history_idx_ >= max_history_length_)
		{
			history_idx_ = 0;
			buffer_full_ = true;
		}
	}

	int getSize() const 
	{
		if (buffer_full_)
		{
			return max_history_length_;
		}
		else 
		{
			return history_idx_;
		}
	}

	data_type_ getData(int idx) const 
	{
		if (idx >= getSize() || idx < 0) 
		{
		    RTT::Logger::In in("DataBufferManagerSupport::getData");
		    Logger::log() << "wrong idx " << idx << Logger::endl;
		}
		int buf_idx = (history_idx_ + idx - getSize() + max_history_length_)%max_history_length_;
		return history_[buf_idx];
	}

	bool isValid(int idx) const 
	{
		if (idx >= getSize() || idx < 0) 
		{
		    RTT::Logger::In in("DataBufferManagerSupport::isValid");
		    Logger::log() << "wrong idx " << idx << Logger::endl;
		}
		int buf_idx = (history_idx_ + idx - getSize() + max_history_length_)%max_history_length_;
		return history_valid_[buf_idx];
	}

private:
	int history_idx_;
	const int max_history_length_;
	RTT::InputPort<data_type_> port_;
	std::vector<data_type_> history_;
	std::vector<bool> history_valid_;
	bool buffer_full_;
};
