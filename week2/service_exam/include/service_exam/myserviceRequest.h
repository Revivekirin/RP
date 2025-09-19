namespace service_exam
{
    template <class ContainerAllocator>
    struct myserviceRequest_
    {
        typedef myserviceRequest_<ContainerAllocator> Type;

        myserviceRequest_()
        : a(0)
        , b(0) {
        }
        myserviceRequest_(const ContainerAllocator& _alloc)
        : a(0)
        , b(0) {
            (void)_alloc;
        }

        typedef int64_t _a_type;
        _a_type a;

        typedef int64_t _b_type;
        _b_type b;
    }
}