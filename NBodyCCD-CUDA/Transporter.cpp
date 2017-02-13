//
// Created by ZhangHanlin on 24/12/15.
//

#include "Transporter.h"
#include <fstream>
#include <sstream>
using namespace std;

Transporter::Transporter() {
    MPI_Comm_rank(MPI_COMM_WORLD, &proc_rank);
    MPI_Comm_size(MPI_COMM_WORLD, &npes);
    ostringstream stringStream;
    stringStream << "record/rank-" << proc_rank << ".txt";
    filePath = stringStream.str();
    ofstream myfile;
    myfile.open (filePath, fstream::trunc);
    myfile << "Rank: " << proc_rank << "\n";
    myfile.close();
}

void Transporter::recordUpdateTime(float time) {
    ofstream myfile;
    myfile.open(filePath, ios_base::app);
    myfile << "Update Time: " << time << "\n";
    myfile.close();
}

void Transporter::recordSubdivideTime(float time) {
    ofstream myfile;
    myfile.open (filePath, ios_base::app);
    myfile << "Subdivide Time: " << time << "\n";
    myfile.close();
}

vector<int> Transporter::gatherCollisionInfo(int count) {
    vector<int> numbersOfCollisions;
    numbersOfCollisions.resize(npes);
    MPI_Allgather(&count, 1, MPI_INT, &numbersOfCollisions[0], 1, MPI_INT, MPI_COMM_WORLD);
    return numbersOfCollisions;
}

vector<int> Transporter::identifyLoadBalancingSpaces(int count) {
    vector<int> numbersOfCollisions;
    numbersOfCollisions = gatherCollisionInfo(count);
    vector<int> spaces_identifiers;
    spaces_identifiers.resize(2);
    int max = numbersOfCollisions[0];
    int max_identifier = 0;
    int min = numbersOfCollisions[0];
    int min_identifier = 0;
    for (int i = 1; i < numbersOfCollisions.size(); ++i) {
        if (numbersOfCollisions[i] > max) {
            max = numbersOfCollisions[i];
            max_identifier = i;
        }
        if (numbersOfCollisions[i] < min) {
            min = numbersOfCollisions[i];
            min_identifier = i;
        }
    }
    spaces_identifiers[0] = max_identifier;
    spaces_identifiers[1] = min_identifier;
    return  spaces_identifiers;
}

vector<float> Transporter::transferCells(vector<int> spaceIdentifiers, vector<float> data) {
    vector<float> result;
    int numOfData;
    //cout << "rank: " << proc_rank << " - " << spaceIdentifiers[0] << spaceIdentifiers[1] << endl;
    if (proc_rank == spaceIdentifiers[0]) {
        // send
        numOfData = data.size();
        //cout << "2. rank: " << proc_rank << " - " << numOfData << endl;
        MPI_Request* request1 = (MPI_Request *)malloc(sizeof(MPI_Request));
        MPI_Isend(&numOfData, 1, MPI_INT, spaceIdentifiers[1], 0, MPI_COMM_WORLD, request1);
//        MPI_Send(&numOfData, 1, MPI_INT, spaceIdentifiers[1], 1, MPI_COMM_WORLD);
    } else if (proc_rank == spaceIdentifiers[1]) {
        // receive
        //cout << "3. rank: " << proc_rank << endl;

        MPI_Request* request1 = (MPI_Request *)malloc(sizeof(MPI_Request));
        MPI_Irecv(&numOfData, 1, MPI_INT, spaceIdentifiers[0], 0, MPI_COMM_WORLD, request1);
        MPI_Status *status1 = (MPI_Status *)malloc(sizeof(MPI_Status));
        MPI_Wait(request1, status1);
        free(status1);
    }

    MPI_Barrier(MPI_COMM_WORLD);

    if (proc_rank == spaceIdentifiers[0]) {
        // send
        MPI_Request* request2 = (MPI_Request *)malloc(sizeof(MPI_Request));
        MPI_Isend(&data[0], numOfData, MPI_FLOAT, spaceIdentifiers[1], 0, MPI_COMM_WORLD, request2);
//        MPI_Send(&data[0], numOfData, MPI_FLOAT, spaceIdentifiers[1], 0, MPI_COMM_WORLD);
    } else if (proc_rank == spaceIdentifiers[1]) {
        // receive
//        MPI_Status status;
//        MPI_Recv(&numOfData, 1, MPI_INT, spaceIdentifiers[0], 1, MPI_COMM_WORLD, &status);
        result.resize(numOfData);
        //cout << "4. rank: " << proc_rank << " receive: " << numOfData << endl;

        MPI_Request* request2 = (MPI_Request *)malloc(sizeof(MPI_Request));
        MPI_Irecv(&result[0], numOfData, MPI_FLOAT, spaceIdentifiers[0], 0, MPI_COMM_WORLD, request2);
        MPI_Status *status2 = (MPI_Status *)malloc(sizeof(MPI_Status));
        MPI_Wait(request2, status2);
        free(status2);

//        MPI_Recv(&result[0], numOfData, MPI_FLOAT, spaceIdentifiers[0], 0, MPI_COMM_WORLD, &status);
    }
    return result;
}

vector<int> Transporter::dynamicIdentifyLoadBalancingSpaces(int count, int cell_count) {
    vector<int> numbersOfCollisions, numberOfCell;
    numbersOfCollisions = gatherCollisionInfo(count);
    numberOfCell = gatherCollisionInfo(cell_count);
    int number_of_spaces = numbersOfCollisions.size();
    vector<int> spaces_identifiers;
    spaces_identifiers.resize(3);
    vector<int> spaces_sequence;
    spaces_sequence.resize(number_of_spaces);

    int i, j, tmp;
    for (int k = 0; k < number_of_spaces; ++k) {
        spaces_sequence[k] = k;
    }
    for (i = 1; i < number_of_spaces; ++i) {
        j = i;
        while (j>0 && numbersOfCollisions[spaces_sequence[j-1]] > numbersOfCollisions[spaces_sequence[j]]) {
            tmp = spaces_sequence[j];
            spaces_sequence[j] = spaces_sequence[j-1];
            spaces_sequence[j-1] = tmp;
            j--;
        }
    }

    /*for (int l = 0; l < number_of_spaces/2; l++) {
        spaces_identifiers[3*l] = spaces_sequence[number_of_spaces-1-l];
        spaces_identifiers[3*l+1] = spaces_sequence[l];
        spaces_identifiers[3*l+2] = (numbersOfCollisions[spaces_sequence[number_of_spaces-1-l]] -               numbersOfCollisions[spaces_sequence[l]])*numberOfCell[spaces_sequence[number_of_spaces-1-l]]
        /((numbersOfCollisions[spaces_sequence[number_of_spaces-1-l]]*2)+1);
    }*/
    spaces_identifiers[0] = spaces_sequence[number_of_spaces-1];
    //adjacent transfer
    for (int i = 0; i < number_of_spaces-1; i++) {
      if(abs(spaces_identifiers[0]-spaces_sequence[i])==1){
        spaces_identifiers[1] = spaces_sequence[i];
        //if((float)(numbersOfCollisions[spaces_identifiers[0]]-numbersOfCollisions[spaces_identifiers[1]])/(numbersOfCollisions[spaces_identifiers[0]]+1)>0.05)
           spaces_identifiers[2] = 1;
        //else 
           //spaces_identifiers[2] = 0;
	break;
      }
    }
    
   //spaces_identifiers[1] = spaces_sequence[0];
   //if((float)(numbersOfCollisions[spaces_identifiers[0]]-numbersOfCollisions[spaces_identifiers[1]])/(numbersOfCollisions[spaces_identifiers[0]]+1)>0.05)
       //spaces_identifiers[2] = 1;
    //else 
       //spaces_identifiers[2] = 0;

    return  spaces_identifiers;
}

vector<float> Transporter::dynamicTransferCells(vector<int> spaceIdentifiers, vector<float> data) {
    vector<float> result;
    int numOfData;

    int num_of_spaces = spaceIdentifiers.size()/3;
    for (int i = 0; i < num_of_spaces; ++i) {
        if (proc_rank == spaceIdentifiers[3*i]) {
            // send
            numOfData = data.size();
            MPI_Request* request1 = (MPI_Request *)malloc(sizeof(MPI_Request));
            MPI_Isend(&numOfData, 1, MPI_INT, spaceIdentifiers[3*i+1], 0, MPI_COMM_WORLD, request1);
        } else if (proc_rank == spaceIdentifiers[3*i+1]) {
            // receive
            MPI_Request* request1 = (MPI_Request *)malloc(sizeof(MPI_Request));
            MPI_Irecv(&numOfData, 1, MPI_INT, spaceIdentifiers[3*i], 0, MPI_COMM_WORLD, request1);
            MPI_Status *status1 = (MPI_Status *)malloc(sizeof(MPI_Status));
            MPI_Wait(request1, status1);
            free(status1);
        }
    }

    MPI_Barrier(MPI_COMM_WORLD);

    for (int i = 0; i < num_of_spaces; ++i) {
        if (proc_rank == spaceIdentifiers[3*i]) {
            // send
            MPI_Request* request2 = (MPI_Request *)malloc(sizeof(MPI_Request));
            MPI_Isend(&data[0], numOfData, MPI_FLOAT, spaceIdentifiers[3*i+1], 0, MPI_COMM_WORLD, request2);
        } else if (proc_rank == spaceIdentifiers[3*i+1]) {
            // receive
            result.resize(numOfData);
            MPI_Request* request2 = (MPI_Request *)malloc(sizeof(MPI_Request));
            MPI_Irecv(&result[0], numOfData, MPI_FLOAT, spaceIdentifiers[3*i], 0, MPI_COMM_WORLD, request2);
            MPI_Status *status2 = (MPI_Status *)malloc(sizeof(MPI_Status));
            MPI_Wait(request2, status2);
            free(status2);
        }
    }

    return result;
}

vector<int> Transporter::broadcastInfo(vector<int> data) {

    vector<int> numbersOfDataTobeBcast;
    vector<int> result;
    numbersOfDataTobeBcast.resize(npes);
    int count = data.size();
    MPI_Allgather(&count, 1, MPI_INT, &numbersOfDataTobeBcast[0], 1, MPI_INT, MPI_COMM_WORLD);


    for (int i = 0; i < npes; ++i) {
        vector<int> tmp;
        tmp.resize(numbersOfDataTobeBcast[i]);
        if (proc_rank == i) {
            tmp = data;
        }
        MPI_Bcast(&tmp[0], numbersOfDataTobeBcast[i], MPI_INT, i, MPI_COMM_WORLD);
        result.insert(result.end(), tmp.begin(), tmp.end());
    }

    return result;
}

map<int, vector<float>> Transporter::p2pTransport(map<int, vector<float>> data) {
    // temporary solution
    for (int j = 0; j < npes; ++j) {
        data[j].push_back(-1);
    }

    // transfer data length
    int *numbersOfDataTobeSent = new int[npes* sizeof(int)];
    int *numbersOfDataTobeReceived = new int[npes* sizeof(int)];

    for (int l = 0; l < npes; ++l) {
        numbersOfDataTobeSent[l] = data[l].size();
    }

    MPI_Alltoall(numbersOfDataTobeSent, 1, MPI_INT, numbersOfDataTobeReceived, 1, MPI_INT, MPI_COMM_WORLD);

    map<int, vector<float>> incomingAvatars;
    for (int i = 0; i < npes; ++i) {
        if (numbersOfDataTobeReceived[i] != 0 && i != proc_rank) {
            incomingAvatars[i] = vector<float>();
            IncomingRequests[i] = (MPI_Request *)malloc(sizeof(MPI_Request));
        }
    }

    // transfer actual data
    for (map<int, vector<float>>::iterator it = data.begin(); it != data.end(); it++) {
        int key = it->first;  // Get the key
        vector<float> value = it->second;  // Get the value
        if (key != proc_rank) {
            outgoingRequests[key] = (MPI_Request *)malloc(sizeof(MPI_Request));
            MPI_Isend(&(data[key])[0], data[key].size(), MPI_FLOAT, key, 0, MPI_COMM_WORLD, outgoingRequests[key]);
            free(outgoingRequests[key]);
        }
    }

    for (int i = 0; i < npes; ++i) {
        if (numbersOfDataTobeReceived[i] != 0 && i != proc_rank) {
            incomingAvatars[i].resize(numbersOfDataTobeReceived[i]);
            MPI_Irecv(&(incomingAvatars[i])[0], numbersOfDataTobeReceived[i], MPI_FLOAT, i, 0, MPI_COMM_WORLD, IncomingRequests[i]);
        }
    }

    for (int i = 0; i < npes; ++i) {
        if (numbersOfDataTobeReceived[i] != 0 && i != proc_rank) {
            MPI_Status *status = (MPI_Status *)malloc(sizeof(MPI_Status));
            MPI_Wait(IncomingRequests[i], status);
            free(status);
            free(IncomingRequests[i]);
        }
    }

    // temporary solution
    for (int k = 0; k < npes; ++k) {
        if (k != proc_rank) {
            incomingAvatars[k].pop_back();
        }
    }

    return incomingAvatars;
}

//map<int, vector<float>> Transporter::gatherCollisionInfo(map<int, vector<float>> data) {
//
//}
