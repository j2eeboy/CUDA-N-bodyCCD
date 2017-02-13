//
// Created by ZhangHanlin on 24/12/15.
//

#ifndef MPI_API_TRANSPORTER_H
#define MPI_API_TRANSPORTER_H

#include <vector>
#include <map>
#include <mpi.h>

class Transporter {
public:
    // init
    Transporter();

    // properties
    int proc_rank, npes;
    std::string filePath;

    // mpi request
    std::map<int, MPI_Request*> outgoingRequests;
    std::map<int, MPI_Request*> IncomingRequests;

    // methods
    void recordUpdateTime(float time);
    void recordSubdivideTime(float time);

    std::vector<int> gatherCollisionInfo(int count);
    std::vector<int> identifyLoadBalancingSpaces(int count);
    std::vector<float> transferCells(std::vector<int> spaceIdentifiers, std::vector<float> data);
    std::vector<int> dynamicIdentifyLoadBalancingSpaces(int count, int cell_count);
    std::vector<float> dynamicTransferCells(std::vector<int> spaceIdentifiers, std::vector<float> data);

    std::vector<int> broadcastInfo(std::vector<int> data);
    std::map<int, std::vector<float>> p2pTransport(std::map<int, std::vector<float>> data);
    //std::map<int, std::vector<float>> gatherCollisionInfo(std::map<int, std::vector<float>> data);
};


#endif //MPI_API_TRANSPORTER_H
