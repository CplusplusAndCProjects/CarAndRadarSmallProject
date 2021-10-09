#include <iostream>
#include<vector>
using namespace std;

int main1()
{
    unsigned int N =0, T=0, k=0;
    std::vector<int*> A;
    string A_string = "";
    cout << "Nhap so test T:";
    cin>>T;

    cout << "Nhap do dai N: ";
    cin>>N;

    cout << "Nhap do dai toi tieu phan duoc chon k:";
    cin>>k;

    cout << "Nhap so phan va suc cong pha cua moi phan (vi du nhap: 1,2,3,-4,5 )" << endl;
    cout << "A: ";
    cin>>A_string;

    cout << "Du lieu da nhap xong, nhan enter de xem ket qua:"<<endl;
    cin.ignore();
    return 0;
}

int main()
{
    int T,N,K;
    std::vector<int> A;
    std::vector<int> caculate;
    cout<<"Nhap vao so lan Test"<<endl;
    cin>>T;
    for(int i=1;i<=T;i++)
    {
       cout<<"Test lan thu:"<<i<<endl;
       cout<<"Nhap chieu dai N: "<<endl;
       cin>>N;
       cout<<"Do dai toi thieu can chon: "<<endl;
       cin>>K;
       cout<<"Chon ra"<< " " << K << " "<<"phan  tu qua bom"<<endl;
       cout<<"Nhap vao suc cong pha cua tung phan"<<endl;
       for(int i=0;i<N;i++)
            {
                cout<<"A["<<i<<"]=";
                int a=0;
                cin>>a;
                cout<<endl;
                A.push_back(a);
            }

      int max=0;

      while(K<N){

        int tong =0;
        for(int i =0; i<=N-K;i++)
        {
            for(int j =0; j<K;j++)
            {
                tong +=A[i+j];
            }
            if(tong>max)
            {
                max = tong;
            }
            tong = 0;
        }

        K++;
      }
      cout<<"Tong lon nhat la:"<<max<<endl<<endl;
    }
}
