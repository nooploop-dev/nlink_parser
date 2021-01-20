#include <iomanip>
#include <iostream>
#include <numeric>

#include "nprotocol_extracter.h"

using namespace std;

class ProtocolUser : public NProtocolBase {
 public:
  ProtocolUser();

 protected:
  bool Verify(const uint8_t *data) override;
  void HandleData(const uint8_t *data) override;
};

ProtocolUser::ProtocolUser() : NProtocolBase(true, 5, {0x77}) {}

bool ProtocolUser::Verify(const uint8_t *data) {
  uint8_t sum = 0;
  return data[length() - 1] ==
         accumulate(data, data + length() - sizeof(sum), sum);
}

void ProtocolUser::HandleData(const uint8_t *data) {
  cout << "Protocol data extract successfully: \n";
  for (int i = 0; i < length(); ++i) {
    cout << hex << setfill('0') << setw(2) << (int)data[i] << " ";
  }
  cout << endl;
}

int main(int, char **) {
  NProtocolExtracter extracter;

  /**
   * Because NProtocolExtracter will take owner of protocol, so, instantiating
   * objects in the heap.
   * */
  ProtocolUser protocol;

  extracter.AddProtocol(&protocol);

  // Error data does not affect subsequent parsing
  extracter.AddNewData("try error data");

  /**
   * In practical application, the data received at one time may not be
   * complete, but there is no need to worry. Extraction handle will
   * automatically splice the data
   * */
  uint8_t data1[] = {0x77, 0x77, 0x00};
  uint8_t data2[] = {0x00, 0x01, 0x78, 0x77};
  uint8_t data3[] = {0x00, 0x00, 0x02, 0x79};
  cout << "Step 1\n";
  extracter.AddNewData(data1, sizeof(data1));
  cout << "Step 2\n";
  extracter.AddNewData(data2, sizeof(data2));
  cout << "Step 3\n";

  extracter.AddNewData(data3, sizeof(data3));
  cout << "Step 4\n";
  return EXIT_SUCCESS;
}

/** output
 * Step 1
 * Step 2
 * Protocol data extract successfully:
 * 77 00 01 00 78
 * Step 3
 * Protocol data extract successfully:
 * 77 00 02 00 79
 * Step 4
 */