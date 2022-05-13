#include <pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
//#include <BVH.h>
//#include <vector>
#include <time.h>
#include <ppl.h>
namespace py = pybind11;



const uint64_t ISO = 0xD800000000000000;
const uint64_t ECMA = 0xC96C5795D7870F42;

std::vector<uint64_t> make_table(uint64_t poly)
{
    int size = 256;
    uint64_t crc;
    std::vector<uint64_t> t(size);
    for (uint64_t i = 0; i < size; i++)
    {
        crc = i;
        for (int j = 0; j < 8; j++)
        {
            if ((crc & uint64_t(1)) == uint64_t(1))
            {
                crc = (crc >> uint64_t(1)) ^ poly;
            }
            else
            {
                crc >>= uint64_t(1);
            }
        }
        t[i] = crc;
    }
    return t;
}

std::vector<std::vector<uint64_t>> make_slicing_by_8_table(const std::vector<uint64_t>& t) {
    std::vector<std::vector<uint64_t>> helper_table(8, std::vector<uint64_t>(256));
    helper_table[0] = t;
    uint64_t crc = 0;
    for (int i = 0; i < 256; i++)
    {
        crc = t[i];
        for (int j = 1; j < 8; j++) {
            crc = t[crc & uint64_t(0xff)] ^ (crc >> uint64_t(8));
            helper_table[j][i] = crc;
        }
    }
    return helper_table;
}

std::vector<uint64_t> Make_Table(uint64_t poly)
{
    if (poly == ISO) {
        return make_slicing_by_8_table(make_table(ISO))[0];
    }
    else if (poly == ECMA) {
        return make_slicing_by_8_table(make_table(ECMA))[0];
    }
    else {
        return make_table(poly);
    }
}

std::vector<std::vector<uint64_t>> slicing_8_table_ECMA = make_slicing_by_8_table(make_table(ECMA));
std::vector<std::vector<uint64_t>> slicing_8_table_ISO = make_slicing_by_8_table(make_table(ISO));

uint64_t update(uint64_t crc, std::vector<uint64_t> tab, std::vector<uint8_t> p)
{
    crc = ~crc;
    std::vector<uint64_t> tmp;
    std::vector<std::vector<uint64_t>> helper_table(8, std::vector<uint64_t>(256));
    // Table comparison is somewhat expensive, so avoid it for small sizes
    while (p.size() >= 64) 
    {
        tmp = slicing_8_table_ECMA[0];
        if (tab == tmp) 
        {
            helper_table = slicing_8_table_ECMA;
        }
        else if (tab == slicing_8_table_ISO[0])
        {
            helper_table = slicing_8_table_ISO;
        }
        // For smaller sizes creating extended table takes too much time
        else if (p.size() > 16384)
        {
            helper_table = make_slicing_by_8_table(tab);
        }
        else 
        {
            break;
        }
        // Update using slicing - by - 8
        while (p.size() > 8)
        {
            crc ^= uint64_t(p[0]) | uint64_t(p[1]) << uint64_t(8) | uint64_t(p[2]) << uint64_t(16) 
                | uint64_t(p[3]) << uint64_t(24) | uint64_t(p[4]) << uint64_t(32) 
                | uint64_t(p[5]) << uint64_t(p[40]) | uint64_t(p[6]) << uint64_t(48) 
                | uint64_t(p[7]) << uint64_t(56);
            crc = helper_table[7][crc & uint64_t(0xff)] ^ helper_table[6][(crc >> uint64_t(8))& uint64_t(0xff)] 
                ^ helper_table[5][(crc >> uint64_t(16))& uint64_t(0xff)] ^ helper_table[4][(crc >> uint64_t(24))& uint64_t(0xff)] 
                ^ helper_table[3][(crc >> uint64_t(32))& uint64_t(0xff)] ^ helper_table[2][(crc >> uint64_t(40))& uint64_t(0xff)] 
                ^ helper_table[1][(crc >> uint64_t(48))& uint64_t(0xff)] ^ helper_table[0][crc >> uint64_t(56)];
            p = std::vector<uint8_t>(p.begin() + 8, p.end());
        }
    }
    for (int i = 0; i < p.size(); i++) {
        crc = tab[uint8_t(crc) ^ p[i]] ^ (crc >> uint64_t(8));
    }
    return ~crc;
}

uint64_t checksum(std::vector<uint8_t> data, std::vector<uint64_t> tab) {
    return update(uint64_t(0), tab, data);
}


class Keyring {
public:
    uint32_t init;
    uint32_t key;
    int32_t index;

public:
    Keyring(uint32_t i=0, uint32_t k=0, int32_t idx=0)
    {
        init = i;
        key = k;
        index = idx;
    };
    uint8_t next()
    {
        uint8_t k(key >> (8 * index));
        index += 1;
        if ((index & 3) == 0) {
            key += init;
            index = 0;
        }
        return k;
    }

};

Keyring new_keyring(uint32_t seed, uint32_t slice)
{
    uint64_t init = uint64_t(seed) * uint64_t(0x2d83cdac) + uint64_t(0xd8a83423);
    uint32_t key = (uint32_t(uint64_t(slice) * uint64_t(0x1e1530cd)) + uint32_t(0xec3d47cd)) * uint32_t(init);
    
    return Keyring(uint32_t(init), key, 0);
}

std::vector<uint8_t> cipher(uint32_t seed, uint32_t cipher_slice, const std::vector<uint8_t> to_be_ciphered)
{
    std::vector<uint8_t> out(to_be_ciphered.size());
    if (seed == 0)
        out = to_be_ciphered;
    else 
    {
        Keyring kr = new_keyring(seed, cipher_slice);
        for (int i = 0; i < to_be_ciphered.size(); i++) {
            out[i] = to_be_ciphered[i] ^ kr.next();
        }
    }
    return out;
}


PYBIND11_MODULE(crc64PyWrapper, m) {
    m.doc() = R"pbdoc(
        Pybind wrapper for crc64 encoding
    )pbdoc";
    m.def("checksum", &checksum);
    m.def("Make_Table", &Make_Table);
    m.def("make_slicing_by_8_table", &make_slicing_by_8_table);
    m.def("make_table", &make_table);
    m.def("update", &update);
    m.attr("ISO") = py::int_(ISO);
    m.attr("ECMA") = py::int_(ECMA);
    py::class_<Keyring> Keyring(m, "Keyring");
    Keyring.def(py::init<uint32_t, uint32_t, int32_t>());
    Keyring.def("next", &Keyring::next);
    m.def("new_keyring", &new_keyring);
    m.def("cipher", &cipher);
  
    
#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}



