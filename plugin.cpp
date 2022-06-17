// This file is part of gfp-basic3d
// Copyright (C) 2018-2022 Ravi Peters

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include <string.h>
#include <geoflow/gfSharedHeadersHash.h>
#include "register.hpp"

#define WIN_DECLSPEC
#ifdef WIN32
	#define WIN_DECLSPEC __declspec (dllexport)
#endif

extern "C"
{
	WIN_DECLSPEC geoflow::NodeRegister *allocator()
	{
    auto node_register = new geoflow::NodeRegister(GF_PLUGIN_NAME);
    register_nodes(*node_register);
		return node_register;
	}

	WIN_DECLSPEC void deleter(geoflow::NodeRegister *ptr)
	{
		delete ptr;
	}

	WIN_DECLSPEC void get_shared_headers_hash(char *hash)
	{
		strcpy(hash, GF_SHARED_HEADERS_HASH);
	}
}