
#ifndef LASERCAKE_UNIQUE_PTR_FROM_LIBCXX_HPP__
#define LASERCAKE_UNIQUE_PTR_FROM_LIBCXX_HPP__

#ifndef USE_BOOST_CXX11_LIBS
#include <memory>
using std::unique_ptr;
using std::default_delete;

#else

#include <memory>
#include <functional>

#include <boost/utility/declval.hpp>
#include <boost/type_traits/common_type.hpp>
#include <boost/type_traits/is_array.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/type_traits/is_scalar.hpp>
#include <boost/type_traits/is_pointer.hpp>
#include <boost/type_traits/is_convertible.hpp>
#include <boost/type_traits/remove_cv.hpp>
#include <boost/type_traits/remove_reference.hpp>
#include <boost/type_traits/add_lvalue_reference.hpp>
#include <boost/type_traits/conditional.hpp>
//#include <boost/intrusive/pointer_traits.hpp>
#include <boost/compressed_pair.hpp>
#include <boost/utility.hpp> //addressof

#include "cxx11_utils.hpp"

#include "../config.hpp"

// TODO: what if we're actually compiling against an old libcxx?
#ifdef LASERCAKE_GCC_LESS_THAN_4_7
#define _LIBCPP_HAS_NO_TEMPLATE_ALIASES
// This is inaccurate but GCC 4.6 doesn't like something
// about the syntactic interaction between noexcept and default.
#define _LIBCPP_HAS_NO_DEFAULTED_FUNCTIONS
#endif
#define _LIBCPP_INLINE_VISIBILITY
#define _LIBCPP_VISIBLE
#define _LIBCPP_CONSTEXPR constexpr
#define _LIBCPP_EXPLICIT explicit
#define _NOEXCEPT noexcept
#define _NOEXCEPT_ noexcept
#define _VSTD unique_ptr_impl

namespace unique_ptr_impl {
using namespace boost;
using std::less;
using std::auto_ptr;
typedef decltype(nullptr) nullptr_t;
//using boost::intrusive::pointer_traits;

template<typename T>
constexpr T&& forward(typename remove_reference<T>::type& t) noexcept
{ return static_cast<T&&>(t); }
template<typename T>
constexpr T&& forward(typename remove_reference<T>::type&& t) noexcept
{ static_assert(!is_lvalue_reference<T>::value, "incorrectly lvalue ref");
  return static_cast<T&&>(t); }
template<typename T>
constexpr typename remove_reference<T>::type&& move(T&& t) noexcept
{ return static_cast<typename remove_reference<T>::type&&>(t); }

#define enable_if enable_if_c
//template<typename T1, typename T2> using 
#define __compressed_pair compressed_pair



//////////////// BEGIN LIBC++ CODE ///////////////////


struct __any
{
    __any(...);
};


// is_assignable

template <class _Tp, class _Arg>
decltype((_VSTD::declval<_Tp>() = _VSTD::declval<_Arg>(), true_type()))
#ifndef _LIBCPP_HAS_NO_RVALUE_REFERENCES
__is_assignable_test(_Tp&&, _Arg&&);
#else
__is_assignable_test(_Tp, _Arg&);
#endif

template <class _Arg>
false_type
#ifndef _LIBCPP_HAS_NO_RVALUE_REFERENCES
__is_assignable_test(__any, _Arg&&);
#else
__is_assignable_test(__any, _Arg&);
#endif

template <class _Tp, class _Arg, bool = is_void<_Tp>::value || is_void<_Arg>::value>
struct __is_assignable_imp
    : public common_type
        <
            decltype(__is_assignable_test(declval<_Tp>(), declval<_Arg>()))
        >::type {};

template <class _Tp, class _Arg>
struct __is_assignable_imp<_Tp, _Arg, true>
    : public false_type
{
};

template <class _Tp, class _Arg>
struct is_assignable
    : public __is_assignable_imp<_Tp, _Arg> {};





// pointer_traits

template <class _Tp>
struct __has_element_type
{
private:
    struct __two {char __lx; char __lxx;};
    template <class _Up> static __two __test(...);
    template <class _Up> static char __test(typename _Up::element_type* = 0);
public:
    static const bool value = sizeof(__test<_Tp>(0)) == 1;
};

template <class _Ptr, bool = __has_element_type<_Ptr>::value>
struct __pointer_traits_element_type;

template <class _Ptr>
struct __pointer_traits_element_type<_Ptr, true>
{
    typedef typename _Ptr::element_type type;
};

#ifndef _LIBCPP_HAS_NO_VARIADICS

template <template <class, class...> class _Sp, class _Tp, class ..._Args>
struct __pointer_traits_element_type<_Sp<_Tp, _Args...>, true>
{
    typedef typename _Sp<_Tp, _Args...>::element_type type;
};

template <template <class, class...> class _Sp, class _Tp, class ..._Args>
struct __pointer_traits_element_type<_Sp<_Tp, _Args...>, false>
{
    typedef _Tp type;
};

#else  // _LIBCPP_HAS_NO_VARIADICS

template <template <class> class _Sp, class _Tp>
struct __pointer_traits_element_type<_Sp<_Tp>, true>
{
    typedef typename _Sp<_Tp>::element_type type;
};

template <template <class> class _Sp, class _Tp>
struct __pointer_traits_element_type<_Sp<_Tp>, false>
{
    typedef _Tp type;
};

template <template <class, class> class _Sp, class _Tp, class _A0>
struct __pointer_traits_element_type<_Sp<_Tp, _A0>, true>
{
    typedef typename _Sp<_Tp, _A0>::element_type type;
};

template <template <class, class> class _Sp, class _Tp, class _A0>
struct __pointer_traits_element_type<_Sp<_Tp, _A0>, false>
{
    typedef _Tp type;
};

template <template <class, class, class> class _Sp, class _Tp, class _A0, class _A1>
struct __pointer_traits_element_type<_Sp<_Tp, _A0, _A1>, true>
{
    typedef typename _Sp<_Tp, _A0, _A1>::element_type type;
};

template <template <class, class, class> class _Sp, class _Tp, class _A0, class _A1>
struct __pointer_traits_element_type<_Sp<_Tp, _A0, _A1>, false>
{
    typedef _Tp type;
};

template <template <class, class, class, class> class _Sp, class _Tp, class _A0,
                                                           class _A1, class _A2>
struct __pointer_traits_element_type<_Sp<_Tp, _A0, _A1, _A2>, true>
{
    typedef typename _Sp<_Tp, _A0, _A1, _A2>::element_type type;
};

template <template <class, class, class, class> class _Sp, class _Tp, class _A0,
                                                           class _A1, class _A2>
struct __pointer_traits_element_type<_Sp<_Tp, _A0, _A1, _A2>, false>
{
    typedef _Tp type;
};

#endif  // _LIBCPP_HAS_NO_VARIADICS

template <class _Tp>
struct __has_difference_type
{
private:
    struct __two {char __lx; char __lxx;};
    template <class _Up> static __two __test(...);
    template <class _Up> static char __test(typename _Up::difference_type* = 0);
public:
    static const bool value = sizeof(__test<_Tp>(0)) == 1;
};

template <class _Ptr, bool = __has_difference_type<_Ptr>::value>
struct __pointer_traits_difference_type
{
    typedef ptrdiff_t type;
};

template <class _Ptr>
struct __pointer_traits_difference_type<_Ptr, true>
{
    typedef typename _Ptr::difference_type type;
};

template <class _Tp, class _Up>
struct __has_rebind
{
private:
    struct __two {char __lx; char __lxx;};
    template <class _Xp> static __two __test(...);
    template <class _Xp> static char __test(typename _Xp::template rebind<_Up>* = 0);
public:
    static const bool value = sizeof(__test<_Tp>(0)) == 1;
};

template <class _Tp, class _Up, bool = __has_rebind<_Tp, _Up>::value>
struct __pointer_traits_rebind
{
#ifndef _LIBCPP_HAS_NO_TEMPLATE_ALIASES
    typedef typename _Tp::template rebind<_Up> type;
#else
    typedef typename _Tp::template rebind<_Up>::other type;
#endif
};

#ifndef _LIBCPP_HAS_NO_VARIADICS

template <template <class, class...> class _Sp, class _Tp, class ..._Args, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp, _Args...>, _Up, true>
{
#ifndef _LIBCPP_HAS_NO_TEMPLATE_ALIASES
    typedef typename _Sp<_Tp, _Args...>::template rebind<_Up> type;
#else
    typedef typename _Sp<_Tp, _Args...>::template rebind<_Up>::other type;
#endif
};

template <template <class, class...> class _Sp, class _Tp, class ..._Args, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp, _Args...>, _Up, false>
{
    typedef _Sp<_Up, _Args...> type;
};

#else  // _LIBCPP_HAS_NO_VARIADICS

template <template <class> class _Sp, class _Tp, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp>, _Up, true>
{
#ifndef _LIBCPP_HAS_NO_TEMPLATE_ALIASES
    typedef typename _Sp<_Tp>::template rebind<_Up> type;
#else
    typedef typename _Sp<_Tp>::template rebind<_Up>::other type;
#endif
};

template <template <class> class _Sp, class _Tp, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp>, _Up, false>
{
    typedef _Sp<_Up> type;
};

template <template <class, class> class _Sp, class _Tp, class _A0, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp, _A0>, _Up, true>
{
#ifndef _LIBCPP_HAS_NO_TEMPLATE_ALIASES
    typedef typename _Sp<_Tp, _A0>::template rebind<_Up> type;
#else
    typedef typename _Sp<_Tp, _A0>::template rebind<_Up>::other type;
#endif
};

template <template <class, class> class _Sp, class _Tp, class _A0, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp, _A0>, _Up, false>
{
    typedef _Sp<_Up, _A0> type;
};

template <template <class, class, class> class _Sp, class _Tp, class _A0,
                                         class _A1, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp, _A0, _A1>, _Up, true>
{
#ifndef _LIBCPP_HAS_NO_TEMPLATE_ALIASES
    typedef typename _Sp<_Tp, _A0, _A1>::template rebind<_Up> type;
#else
    typedef typename _Sp<_Tp, _A0, _A1>::template rebind<_Up>::other type;
#endif
};

template <template <class, class, class> class _Sp, class _Tp, class _A0,
                                         class _A1, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp, _A0, _A1>, _Up, false>
{
    typedef _Sp<_Up, _A0, _A1> type;
};

template <template <class, class, class, class> class _Sp, class _Tp, class _A0,
                                                class _A1, class _A2, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp, _A0, _A1, _A2>, _Up, true>
{
#ifndef _LIBCPP_HAS_NO_TEMPLATE_ALIASES
    typedef typename _Sp<_Tp, _A0, _A1, _A2>::template rebind<_Up> type;
#else
    typedef typename _Sp<_Tp, _A0, _A1, _A2>::template rebind<_Up>::other type;
#endif
};

template <template <class, class, class, class> class _Sp, class _Tp, class _A0,
                                                class _A1, class _A2, class _Up>
struct __pointer_traits_rebind<_Sp<_Tp, _A0, _A1, _A2>, _Up, false>
{
    typedef _Sp<_Up, _A0, _A1, _A2> type;
};

#endif  // _LIBCPP_HAS_NO_VARIADICS

template <class _Ptr>
struct _LIBCPP_VISIBLE pointer_traits
{
    typedef _Ptr                                                     pointer;
    typedef typename __pointer_traits_element_type<pointer>::type    element_type;
    typedef typename __pointer_traits_difference_type<pointer>::type difference_type;

#ifndef _LIBCPP_HAS_NO_TEMPLATE_ALIASES
    template <class _Up> using rebind = typename __pointer_traits_rebind<pointer, _Up>::type;
#else
    template <class _Up> struct rebind
        {typedef typename __pointer_traits_rebind<pointer, _Up>::type other;};
#endif  // _LIBCPP_HAS_NO_TEMPLATE_ALIASES

private:
    struct __nat {};
public:
    _LIBCPP_INLINE_VISIBILITY
    static pointer pointer_to(typename conditional<is_void<element_type>::value,
                                           __nat, element_type>::type& __r)
        {return pointer::pointer_to(__r);}
};

template <class _Tp>
struct _LIBCPP_VISIBLE pointer_traits<_Tp*>
{
    typedef _Tp*      pointer;
    typedef _Tp       element_type;
    typedef ptrdiff_t difference_type;

#ifndef _LIBCPP_HAS_NO_TEMPLATE_ALIASES
    template <class _Up> using rebind = _Up*;
#else
    template <class _Up> struct rebind {typedef _Up* other;};
#endif

private:
    struct __nat {};
public:
    _LIBCPP_INLINE_VISIBILITY
    static pointer pointer_to(typename conditional<is_void<element_type>::value,
                                      __nat, element_type>::type& __r) _NOEXCEPT
        {return _VSTD::addressof(__r);}
};

// allocator_traits

namespace __has_pointer_type_imp
{
    struct __two {char __lx; char __lxx;};
    template <class _Up> static __two test(...);
    template <class _Up> static char test(typename _Up::pointer* = 0);
}

template <class _Tp>
struct __has_pointer_type
    : public integral_constant<bool, sizeof(__has_pointer_type_imp::test<_Tp>(0)) == 1>
{
};

namespace __pointer_type_imp
{

template <class _Tp, class _Dp, bool = __has_pointer_type<_Dp>::value>
struct __pointer_type
{
    typedef typename _Dp::pointer type;
};

template <class _Tp, class _Dp>
struct __pointer_type<_Tp, _Dp, false>
{
    typedef _Tp* type;
};

}  // __pointer_type_imp

template <class _Tp, class _Dp>
struct __pointer_type
{
    typedef typename __pointer_type_imp::__pointer_type<_Tp, typename remove_reference<_Dp>::type>::type type;
};

template <class _Tp>
struct __has_const_pointer
{
private:
    struct __two {char __lx; char __lxx;};
    template <class _Up> static __two __test(...);
    template <class _Up> static char __test(typename _Up::const_pointer* = 0);
public:
    static const bool value = sizeof(__test<_Tp>(0)) == 1;
};

// __same_or_less_cv_qualified

template <class _Ptr1, class _Ptr2,
          bool = is_same<typename remove_cv<typename pointer_traits<_Ptr1>::element_type>::type,
                         typename remove_cv<typename pointer_traits<_Ptr2>::element_type>::type
                        >::value
         >
struct __same_or_less_cv_qualified_imp
    : is_convertible<_Ptr1, _Ptr2> {};

template <class _Ptr1, class _Ptr2>
struct __same_or_less_cv_qualified_imp<_Ptr1, _Ptr2, false>
    : false_type {};

template <class _Ptr1, class _Ptr2, bool = is_scalar<_Ptr1>::value &&
                                         !is_pointer<_Ptr1>::value>
struct __same_or_less_cv_qualified
    : __same_or_less_cv_qualified_imp<_Ptr1, _Ptr2> {};

template <class _Ptr1, class _Ptr2>
struct __same_or_less_cv_qualified<_Ptr1, _Ptr2, true>
    : false_type {};

// default_delete

template <class _Tp>
struct _LIBCPP_VISIBLE default_delete
{
#ifndef _LIBCPP_HAS_NO_DEFAULTED_FUNCTIONS
    _LIBCPP_INLINE_VISIBILITY _LIBCPP_CONSTEXPR default_delete() _NOEXCEPT = default;
#else
    _LIBCPP_INLINE_VISIBILITY _LIBCPP_CONSTEXPR default_delete() _NOEXCEPT {}
#endif
    template <class _Up>
        _LIBCPP_INLINE_VISIBILITY default_delete(const default_delete<_Up>&,
             typename enable_if<is_convertible<_Up*, _Tp*>::value>::type* = 0) _NOEXCEPT {}
    _LIBCPP_INLINE_VISIBILITY void operator() (_Tp* __ptr) const _NOEXCEPT
        {
            static_assert(sizeof(_Tp) > 0, "default_delete can not delete incomplete type");
            delete __ptr;
        }
};

template <class _Tp>
struct _LIBCPP_VISIBLE default_delete<_Tp[]>
{
public:
#ifndef _LIBCPP_HAS_NO_DEFAULTED_FUNCTIONS
    _LIBCPP_INLINE_VISIBILITY _LIBCPP_CONSTEXPR default_delete() _NOEXCEPT = default;
#else
    _LIBCPP_INLINE_VISIBILITY _LIBCPP_CONSTEXPR default_delete() _NOEXCEPT {}
#endif
    template <class _Up>
        _LIBCPP_INLINE_VISIBILITY default_delete(const default_delete<_Up[]>&,
             typename enable_if<__same_or_less_cv_qualified<_Up*, _Tp*>::value>::type* = 0) _NOEXCEPT {}
    template <class _Up>
        _LIBCPP_INLINE_VISIBILITY
        void operator() (_Up* __ptr,
                         typename enable_if<__same_or_less_cv_qualified<_Up*, _Tp*>::value>::type* = 0) const _NOEXCEPT
        {
            static_assert(sizeof(_Tp) > 0, "default_delete can not delete incomplete type");
            delete [] __ptr;
        }
};

template <class _Tp, class _Dp = default_delete<_Tp> >
class _LIBCPP_VISIBLE unique_ptr
{
public:
    typedef _Tp element_type;
    typedef _Dp deleter_type;
    typedef typename __pointer_type<_Tp, deleter_type>::type pointer;
private:
    __compressed_pair<pointer, deleter_type> __ptr_;

#ifdef _LIBCPP_HAS_NO_RVALUE_REFERENCES
    unique_ptr(unique_ptr&);
    template <class _Up, class _Ep>
        unique_ptr(unique_ptr<_Up, _Ep>&);
    unique_ptr& operator=(unique_ptr&);
    template <class _Up, class _Ep>
        unique_ptr& operator=(unique_ptr<_Up, _Ep>&);
#endif  // _LIBCPP_HAS_NO_RVALUE_REFERENCES

    struct __nat {int __for_bool_;};

    typedef       typename remove_reference<deleter_type>::type& _Dp_reference;
    typedef const typename remove_reference<deleter_type>::type& _Dp_const_reference;
public:
    _LIBCPP_INLINE_VISIBILITY _LIBCPP_CONSTEXPR unique_ptr() _NOEXCEPT
        : __ptr_(pointer())
        {
            static_assert(!is_pointer<deleter_type>::value,
                "unique_ptr constructed with null function pointer deleter");
        }
    _LIBCPP_INLINE_VISIBILITY _LIBCPP_CONSTEXPR unique_ptr(nullptr_t) _NOEXCEPT
        : __ptr_(pointer())
        {
            static_assert(!is_pointer<deleter_type>::value,
                "unique_ptr constructed with null function pointer deleter");
        }
    _LIBCPP_INLINE_VISIBILITY explicit unique_ptr(pointer __p) _NOEXCEPT
        : __ptr_(_VSTD::move(__p))
        {
            static_assert(!is_pointer<deleter_type>::value,
                "unique_ptr constructed with null function pointer deleter");
        }

#ifndef _LIBCPP_HAS_NO_RVALUE_REFERENCES
    _LIBCPP_INLINE_VISIBILITY unique_ptr(pointer __p, typename conditional<
                                        is_reference<deleter_type>::value,
                                        deleter_type,
                                        typename add_lvalue_reference<const deleter_type>::type>::type __d)
             _NOEXCEPT
        : __ptr_(__p, __d) {}

    _LIBCPP_INLINE_VISIBILITY unique_ptr(pointer __p, typename remove_reference<deleter_type>::type&& __d)
             _NOEXCEPT
        : __ptr_(__p, _VSTD::move(__d))
        {
            static_assert(!is_reference<deleter_type>::value, "rvalue deleter bound to reference");
        }
    _LIBCPP_INLINE_VISIBILITY unique_ptr(unique_ptr&& __u) _NOEXCEPT
        : __ptr_(__u.release(), _VSTD::forward<deleter_type>(__u.get_deleter())) {}
    template <class _Up, class _Ep>
        _LIBCPP_INLINE_VISIBILITY
        unique_ptr(unique_ptr<_Up, _Ep>&& __u,
                   typename enable_if
                      <
                        !is_array<_Up>::value &&
                         is_convertible<typename unique_ptr<_Up, _Ep>::pointer, pointer>::value &&
                         is_convertible<_Ep, deleter_type>::value &&
                         (
                            !is_reference<deleter_type>::value ||
                            is_same<deleter_type, _Ep>::value
                         ),
                         __nat
                      >::type = __nat()) _NOEXCEPT
            : __ptr_(__u.release(), _VSTD::forward<_Ep>(__u.get_deleter())) {}

    template <class _Up>
        _LIBCPP_INLINE_VISIBILITY unique_ptr(auto_ptr<_Up>&& __p,
                typename enable_if<
                                      is_convertible<_Up*, _Tp*>::value &&
                                      is_same<_Dp, default_delete<_Tp> >::value,
                                      __nat
                                  >::type = __nat()) _NOEXCEPT
            : __ptr_(__p.release())
            {
            }

        _LIBCPP_INLINE_VISIBILITY unique_ptr& operator=(unique_ptr&& __u) _NOEXCEPT
            {
                reset(__u.release());
                __ptr_.second() = _VSTD::forward<deleter_type>(__u.get_deleter());
                return *this;
            }

        template <class _Up, class _Ep>
            _LIBCPP_INLINE_VISIBILITY
            typename enable_if
            <
                !is_array<_Up>::value &&
                is_convertible<typename unique_ptr<_Up, _Ep>::pointer, pointer>::value &&
                is_assignable<deleter_type&, _Ep&&>::value,
                unique_ptr&
            >::type
            operator=(unique_ptr<_Up, _Ep>&& __u) _NOEXCEPT
            {
                reset(__u.release());
                __ptr_.second() = _VSTD::forward<_Ep>(__u.get_deleter());
                return *this;
            }
#else  // _LIBCPP_HAS_NO_RVALUE_REFERENCES

    _LIBCPP_INLINE_VISIBILITY operator __rv<unique_ptr>()
    {
        return __rv<unique_ptr>(*this);
    }

    _LIBCPP_INLINE_VISIBILITY unique_ptr(__rv<unique_ptr> __u)
        : __ptr_(__u->release(), _VSTD::forward<deleter_type>(__u->get_deleter())) {}

    template <class _Up, class _Ep>
    _LIBCPP_INLINE_VISIBILITY unique_ptr& operator=(unique_ptr<_Up, _Ep> __u)
    {
        reset(__u.release());
        __ptr_.second() = _VSTD::forward<deleter_type>(__u.get_deleter());
        return *this;
    }

    _LIBCPP_INLINE_VISIBILITY unique_ptr(pointer __p, deleter_type __d)
        : __ptr_(_VSTD::move(__p), _VSTD::move(__d)) {}

    template <class _Up>
        _LIBCPP_INLINE_VISIBILITY
                typename enable_if<
                                      is_convertible<_Up*, _Tp*>::value &&
                                      is_same<_Dp, default_delete<_Tp> >::value,
                                      unique_ptr&
                                  >::type
        operator=(auto_ptr<_Up> __p)
            {reset(__p.release()); return *this;}

#endif  // _LIBCPP_HAS_NO_RVALUE_REFERENCES
    _LIBCPP_INLINE_VISIBILITY ~unique_ptr() {reset();}

    _LIBCPP_INLINE_VISIBILITY unique_ptr& operator=(nullptr_t) _NOEXCEPT
    {
        reset();
        return *this;
    }

    _LIBCPP_INLINE_VISIBILITY typename add_lvalue_reference<_Tp>::type operator*() const
        {return *__ptr_.first();}
    _LIBCPP_INLINE_VISIBILITY pointer operator->() const _NOEXCEPT {return __ptr_.first();}
    _LIBCPP_INLINE_VISIBILITY pointer get() const _NOEXCEPT {return __ptr_.first();}
    _LIBCPP_INLINE_VISIBILITY       _Dp_reference get_deleter() _NOEXCEPT
        {return __ptr_.second();}
    _LIBCPP_INLINE_VISIBILITY _Dp_const_reference get_deleter() const _NOEXCEPT
        {return __ptr_.second();}
    _LIBCPP_INLINE_VISIBILITY
        _LIBCPP_EXPLICIT operator bool() const _NOEXCEPT
        {return __ptr_.first() != nullptr;}

    _LIBCPP_INLINE_VISIBILITY pointer release() _NOEXCEPT
    {
        pointer __t = __ptr_.first();
        __ptr_.first() = pointer();
        return __t;
    }

    _LIBCPP_INLINE_VISIBILITY void reset(pointer __p = pointer()) _NOEXCEPT
    {
        pointer __tmp = __ptr_.first();
        __ptr_.first() = __p;
        if (__tmp)
            __ptr_.second()(__tmp);
    }

    _LIBCPP_INLINE_VISIBILITY void swap(unique_ptr& __u) _NOEXCEPT
        {__ptr_.swap(__u.__ptr_);}
};

template <class _Tp, class _Dp>
class _LIBCPP_VISIBLE unique_ptr<_Tp[], _Dp>
{
public:
    typedef _Tp element_type;
    typedef _Dp deleter_type;
    typedef typename __pointer_type<_Tp, deleter_type>::type pointer;
private:
    __compressed_pair<pointer, deleter_type> __ptr_;

#ifdef _LIBCPP_HAS_NO_RVALUE_REFERENCES
    unique_ptr(unique_ptr&);
    template <class _Up>
        unique_ptr(unique_ptr<_Up>&);
    unique_ptr& operator=(unique_ptr&);
    template <class _Up>
        unique_ptr& operator=(unique_ptr<_Up>&);
#endif  // _LIBCPP_HAS_NO_RVALUE_REFERENCES

    struct __nat {int __for_bool_;};

    typedef       typename remove_reference<deleter_type>::type& _Dp_reference;
    typedef const typename remove_reference<deleter_type>::type& _Dp_const_reference;
public:
    _LIBCPP_INLINE_VISIBILITY _LIBCPP_CONSTEXPR unique_ptr() _NOEXCEPT
        : __ptr_(pointer())
        {
            static_assert(!is_pointer<deleter_type>::value,
                "unique_ptr constructed with null function pointer deleter");
        }
    _LIBCPP_INLINE_VISIBILITY _LIBCPP_CONSTEXPR unique_ptr(nullptr_t) _NOEXCEPT
        : __ptr_(pointer())
        {
            static_assert(!is_pointer<deleter_type>::value,
                "unique_ptr constructed with null function pointer deleter");
        }
#ifndef _LIBCPP_HAS_NO_RVALUE_REFERENCES
    template <class _Pp,
              class = typename enable_if<__same_or_less_cv_qualified<_Pp, pointer>::value>::type
             >
    _LIBCPP_INLINE_VISIBILITY explicit unique_ptr(_Pp __p) _NOEXCEPT
        : __ptr_(__p)
        {
            static_assert(!is_pointer<deleter_type>::value,
                "unique_ptr constructed with null function pointer deleter");
        }

    template <class _Pp,
              class = typename enable_if<__same_or_less_cv_qualified<_Pp, pointer>::value>::type
             >
    _LIBCPP_INLINE_VISIBILITY unique_ptr(_Pp __p, typename conditional<
                                       is_reference<deleter_type>::value,
                                       deleter_type,
                                       typename add_lvalue_reference<const deleter_type>::type>::type __d)
             _NOEXCEPT
        : __ptr_(__p, __d) {}

    _LIBCPP_INLINE_VISIBILITY unique_ptr(nullptr_t, typename conditional<
                                       is_reference<deleter_type>::value,
                                       deleter_type,
                                       typename add_lvalue_reference<const deleter_type>::type>::type __d)
             _NOEXCEPT
        : __ptr_(pointer(), __d) {}

    template <class _Pp,
              class = typename enable_if<__same_or_less_cv_qualified<_Pp, pointer>::value>::type
             >
    _LIBCPP_INLINE_VISIBILITY unique_ptr(_Pp __p, typename remove_reference<deleter_type>::type&& __d)
             _NOEXCEPT
        : __ptr_(__p, _VSTD::move(__d))
        {
            static_assert(!is_reference<deleter_type>::value, "rvalue deleter bound to reference");
        }

    _LIBCPP_INLINE_VISIBILITY unique_ptr(nullptr_t, typename remove_reference<deleter_type>::type&& __d)
             _NOEXCEPT
        : __ptr_(pointer(), _VSTD::move(__d))
        {
            static_assert(!is_reference<deleter_type>::value, "rvalue deleter bound to reference");
        }

    _LIBCPP_INLINE_VISIBILITY unique_ptr(unique_ptr&& __u) _NOEXCEPT
        : __ptr_(__u.release(), _VSTD::forward<deleter_type>(__u.get_deleter())) {}

    _LIBCPP_INLINE_VISIBILITY unique_ptr& operator=(unique_ptr&& __u) _NOEXCEPT
        {
            reset(__u.release());
            __ptr_.second() = _VSTD::forward<deleter_type>(__u.get_deleter());
            return *this;
        }

    template <class _Up, class _Ep>
        _LIBCPP_INLINE_VISIBILITY
        unique_ptr(unique_ptr<_Up, _Ep>&& __u,
                   typename enable_if
                            <
                                is_array<_Up>::value &&
                                __same_or_less_cv_qualified<typename unique_ptr<_Up, _Ep>::pointer, pointer>::value
                                && is_convertible<_Ep, deleter_type>::value &&
                                (
                                    !is_reference<deleter_type>::value ||
                                    is_same<deleter_type, _Ep>::value
                                ),
                                __nat
                            >::type = __nat()
                  ) _NOEXCEPT
        : __ptr_(__u.release(), _VSTD::forward<deleter_type>(__u.get_deleter())) {}


        template <class _Up, class _Ep>
            _LIBCPP_INLINE_VISIBILITY
            typename enable_if
            <
                is_array<_Up>::value &&
                __same_or_less_cv_qualified<typename unique_ptr<_Up, _Ep>::pointer, pointer>::value &&
                is_assignable<deleter_type&, _Ep&&>::value,
                unique_ptr&
            >::type
            operator=(unique_ptr<_Up, _Ep>&& __u) _NOEXCEPT
            {
                reset(__u.release());
                __ptr_.second() = _VSTD::forward<_Ep>(__u.get_deleter());
                return *this;
            }
#else  // _LIBCPP_HAS_NO_RVALUE_REFERENCES

    _LIBCPP_INLINE_VISIBILITY explicit unique_ptr(pointer __p)
        : __ptr_(__p)
        {
            static_assert(!is_pointer<deleter_type>::value,
                "unique_ptr constructed with null function pointer deleter");
        }

    _LIBCPP_INLINE_VISIBILITY unique_ptr(pointer __p, deleter_type __d)
        : __ptr_(__p, _VSTD::forward<deleter_type>(__d)) {}

    _LIBCPP_INLINE_VISIBILITY unique_ptr(nullptr_t, deleter_type __d)
        : __ptr_(pointer(), _VSTD::forward<deleter_type>(__d)) {}

    _LIBCPP_INLINE_VISIBILITY operator __rv<unique_ptr>()
    {
        return __rv<unique_ptr>(*this);
    }

    _LIBCPP_INLINE_VISIBILITY unique_ptr(__rv<unique_ptr> __u)
        : __ptr_(__u->release(), _VSTD::forward<deleter_type>(__u->get_deleter())) {}

    _LIBCPP_INLINE_VISIBILITY unique_ptr& operator=(__rv<unique_ptr> __u)
    {
        reset(__u->release());
        __ptr_.second() = _VSTD::forward<deleter_type>(__u->get_deleter());
        return *this;
    }

#endif  // _LIBCPP_HAS_NO_RVALUE_REFERENCES
    _LIBCPP_INLINE_VISIBILITY ~unique_ptr() {reset();}

    _LIBCPP_INLINE_VISIBILITY unique_ptr& operator=(nullptr_t) _NOEXCEPT
    {
        reset();
        return *this;
    }

    _LIBCPP_INLINE_VISIBILITY typename add_lvalue_reference<_Tp>::type operator[](size_t __i) const
        {return __ptr_.first()[__i];}
    _LIBCPP_INLINE_VISIBILITY pointer get() const _NOEXCEPT {return __ptr_.first();}
    _LIBCPP_INLINE_VISIBILITY       _Dp_reference get_deleter() _NOEXCEPT
        {return __ptr_.second();}
    _LIBCPP_INLINE_VISIBILITY _Dp_const_reference get_deleter() const _NOEXCEPT
        {return __ptr_.second();}
    _LIBCPP_INLINE_VISIBILITY
        _LIBCPP_EXPLICIT operator bool() const _NOEXCEPT
        {return __ptr_.first() != nullptr;}

    _LIBCPP_INLINE_VISIBILITY pointer release() _NOEXCEPT
    {
        pointer __t = __ptr_.first();
        __ptr_.first() = pointer();
        return __t;
    }

#ifndef _LIBCPP_HAS_NO_RVALUE_REFERENCES
    template <class _Pp,
              class = typename enable_if<__same_or_less_cv_qualified<_Pp, pointer>::value>::type
             >
    _LIBCPP_INLINE_VISIBILITY void reset(_Pp __p) _NOEXCEPT
    {
        pointer __tmp = __ptr_.first();
        __ptr_.first() = __p;
        if (__tmp)
            __ptr_.second()(__tmp);
    }
    _LIBCPP_INLINE_VISIBILITY void reset(nullptr_t) _NOEXCEPT
    {
        pointer __tmp = __ptr_.first();
        __ptr_.first() = nullptr;
        if (__tmp)
            __ptr_.second()(__tmp);
    }
    _LIBCPP_INLINE_VISIBILITY void reset() _NOEXCEPT
    {
        pointer __tmp = __ptr_.first();
        __ptr_.first() = nullptr;
        if (__tmp)
            __ptr_.second()(__tmp);
    }
#else  // _LIBCPP_HAS_NO_RVALUE_REFERENCES
    _LIBCPP_INLINE_VISIBILITY void reset(pointer __p = pointer())
    {
        pointer __tmp = __ptr_.first();
        __ptr_.first() = __p;
        if (__tmp)
            __ptr_.second()(__tmp);
    }
#endif  // _LIBCPP_HAS_NO_RVALUE_REFERENCES

    _LIBCPP_INLINE_VISIBILITY void swap(unique_ptr& __u) {__ptr_.swap(__u.__ptr_);}
private:

#ifdef _LIBCPP_HAS_NO_RVALUE_REFERENCES
    template <class _Up>
        explicit unique_ptr(_Up);
    template <class _Up>
        unique_ptr(_Up __u,
                   typename conditional<
                                       is_reference<deleter_type>::value,
                                       deleter_type,
                                       typename add_lvalue_reference<const deleter_type>::type>::type,
                   typename enable_if
                      <
                         is_convertible<_Up, pointer>::value,
                         __nat
                      >::type = __nat());
#endif  // _LIBCPP_HAS_NO_RVALUE_REFERENCES
};

template <class _Tp, class _Dp>
inline _LIBCPP_INLINE_VISIBILITY
void
swap(unique_ptr<_Tp, _Dp>& __x, unique_ptr<_Tp, _Dp>& __y) _NOEXCEPT {__x.swap(__y);}

template <class _T1, class _D1, class _T2, class _D2>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator==(const unique_ptr<_T1, _D1>& __x, const unique_ptr<_T2, _D2>& __y) {return __x.get() == __y.get();}

template <class _T1, class _D1, class _T2, class _D2>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator!=(const unique_ptr<_T1, _D1>& __x, const unique_ptr<_T2, _D2>& __y) {return !(__x == __y);}

template <class _T1, class _D1, class _T2, class _D2>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator< (const unique_ptr<_T1, _D1>& __x, const unique_ptr<_T2, _D2>& __y)
{
    typedef typename unique_ptr<_T1, _D1>::pointer _P1;
    typedef typename unique_ptr<_T2, _D2>::pointer _P2;
    typedef typename common_type<_P1, _P2>::type _V;
    return less<_V>()(__x.get(), __y.get());
}

template <class _T1, class _D1, class _T2, class _D2>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator> (const unique_ptr<_T1, _D1>& __x, const unique_ptr<_T2, _D2>& __y) {return __y < __x;}

template <class _T1, class _D1, class _T2, class _D2>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator<=(const unique_ptr<_T1, _D1>& __x, const unique_ptr<_T2, _D2>& __y) {return !(__y < __x);}

template <class _T1, class _D1, class _T2, class _D2>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator>=(const unique_ptr<_T1, _D1>& __x, const unique_ptr<_T2, _D2>& __y) {return !(__x < __y);}

template <class _T1, class _D1>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator==(const unique_ptr<_T1, _D1>& __x, nullptr_t) _NOEXCEPT
{
    return !__x;
}

template <class _T1, class _D1>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator==(nullptr_t, const unique_ptr<_T1, _D1>& __x) _NOEXCEPT
{
    return !__x;
}

template <class _T1, class _D1>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator!=(const unique_ptr<_T1, _D1>& __x, nullptr_t) _NOEXCEPT
{
    return static_cast<bool>(__x);
}

template <class _T1, class _D1>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator!=(nullptr_t, const unique_ptr<_T1, _D1>& __x) _NOEXCEPT
{
    return static_cast<bool>(__x);
}

template <class _T1, class _D1>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator<(const unique_ptr<_T1, _D1>& __x, nullptr_t)
{
    typedef typename unique_ptr<_T1, _D1>::pointer _P1;
    return less<_P1>()(__x.get(), nullptr);
}

template <class _T1, class _D1>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator<(nullptr_t, const unique_ptr<_T1, _D1>& __x)
{
    typedef typename unique_ptr<_T1, _D1>::pointer _P1;
    return less<_P1>()(nullptr, __x.get());
}

template <class _T1, class _D1>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator>(const unique_ptr<_T1, _D1>& __x, nullptr_t)
{
    return nullptr < __x;
}

template <class _T1, class _D1>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator>(nullptr_t, const unique_ptr<_T1, _D1>& __x)
{
    return __x < nullptr;
}

template <class _T1, class _D1>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator<=(const unique_ptr<_T1, _D1>& __x, nullptr_t)
{
    return !(nullptr < __x);
}

template <class _T1, class _D1>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator<=(nullptr_t, const unique_ptr<_T1, _D1>& __x)
{
    return !(__x < nullptr);
}

template <class _T1, class _D1>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator>=(const unique_ptr<_T1, _D1>& __x, nullptr_t)
{
    return !(__x < nullptr);
}

template <class _T1, class _D1>
inline _LIBCPP_INLINE_VISIBILITY
bool
operator>=(nullptr_t, const unique_ptr<_T1, _D1>& __x)
{
    return !(nullptr < __x);
}

#ifdef _LIBCPP_HAS_NO_RVALUE_REFERENCES

template <class _Tp, class _Dp>
inline _LIBCPP_INLINE_VISIBILITY
unique_ptr<_Tp, _Dp>
move(unique_ptr<_Tp, _Dp>& __t)
{
    return unique_ptr<_Tp, _Dp>(__rv<unique_ptr<_Tp, _Dp> >(__t));
}

#endif

//////////////// END LIBC++ CODE ///////////////////

#undef enable_if

}
using unique_ptr_impl::unique_ptr;
using unique_ptr_impl::default_delete;

#endif

#endif

