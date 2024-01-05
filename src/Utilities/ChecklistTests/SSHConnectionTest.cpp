//
// Created by abiel on 2/12/22.
//

#include "SSHConnectionTest.h"
#include <wpi/Logger.h>
#include <wpinet/uv/Tcp.h>

SSHConnectionTest::SSHConnectionTest(const std::string &hostname) : ChecklistItem(fmt::format("SSHConnectionTest: {}", hostname), false) {
    this->hostname = hostname;
}

void SSHConnectionTest::Execute() {
    auto logger = wpi::Logger(
            [](unsigned int level, const char *file, unsigned int line, const char* msg){;}
    );
//
//    auto conn = wpi::uv::Tcp::Connect(
//            hostname.c_str(),
//            22);
    auto conn = nullptr;

    assertTest("Connection successful", conn != nullptr);
    finished = true;
}

bool SSHConnectionTest::IsFinished() {
    return finished;
}