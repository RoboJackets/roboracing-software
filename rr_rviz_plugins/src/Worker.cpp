#include "Worker.hpp"

Worker::Worker() {  // Constructor
    // you could copy data from constructor arguments to internal variables here.
}

Worker::~Worker() {  // Destructor
    // free resources
}

void Worker::process() {  // Process. Start processing data.
    // allocate resources using new here
    qDebug("Hello World!");
    emit finished();
}
