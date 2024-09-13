import 'package:flutter/material.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import 'package:ugv_control_app/home_page.dart';
import 'package:crypto/crypto.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'dart:convert';

class LoginPage extends StatefulWidget {
  const LoginPage({super.key, required this.client});
  final MqttServerClient? client;

  @override
  State<LoginPage> createState() => _LoginPageState(client);
}

class _LoginPageState extends State<LoginPage> {
  MqttServerClient? client;
  bool showSignup = false;
  final TextEditingController _emailController = TextEditingController();
  final TextEditingController _passwordController = TextEditingController();
  final TextEditingController _confirmPasswordController = TextEditingController();

  _LoginPageState(this.client);

  String _hashPassword(String password) {
    final bytes = utf8.encode(password);
    final digest = sha256.convert(bytes);
    return digest.toString();
  }

  Future<void> _login() async {
    try {
      final SharedPreferences prefs = await SharedPreferences.getInstance();
      final String? hashedPassword = prefs.getString(_emailController.text.trim());
  
      if (hashedPassword != null && hashedPassword == _hashPassword(_passwordController.text.trim())) {
        Navigator.push(
          context,
          MaterialPageRoute(builder: (context) => UGVHomePage(client: client!)),
        );
      } else {
        ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Login failed: Invalid credentials')),
        );
      }
}  catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('An error occurred: $e')),
      );
    }
  }


  Future<void> _signup() async {    
    if (_passwordController.text != _confirmPasswordController.text) {
      ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(content: Text('Passwords do not match')),
      );
      return;
    }
    try {
      final SharedPreferences prefs = await SharedPreferences.getInstance();
      final String email = _emailController.text.trim();
      final String hashedPassword = _hashPassword(_passwordController.text.trim());

      await prefs.setString(email, hashedPassword);
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Signup successful')),
      );
      setState(() {
        showSignup = false;
      });
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('An error occurred: $e')),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    final Column content = !showSignup
        ? Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              TextField(
                controller: _emailController,
                decoration: InputDecoration(border: OutlineInputBorder(), hintText: "Email"),
                keyboardType: TextInputType.emailAddress,
              ),
              TextField(
                controller: _passwordController,
                decoration: InputDecoration(border: OutlineInputBorder(), hintText: "Password"),
                obscureText: true,
              ),
              Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  ElevatedButton(onPressed: _login, child: const Text("Login")),
                  ElevatedButton(
                    onPressed: () => setState(() => showSignup = true),
                    child: const Text("Signup"),
                  ),
                ],
              ),
            ],
          )
        : Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              TextField(
                controller: _emailController,
                decoration: InputDecoration(border: OutlineInputBorder(), hintText: "Email"),
                keyboardType: TextInputType.emailAddress,
              ),
              TextField(
                controller: _passwordController,
                decoration: InputDecoration(border: OutlineInputBorder(), hintText: "Password"),
                obscureText: true,
              ),
              TextField(
                controller: _confirmPasswordController,
                decoration: InputDecoration(border: OutlineInputBorder(), hintText: "Confirm Password"),
                obscureText: true,
              ),
              Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  ElevatedButton(onPressed: _signup, child: const Text("Signup")),
                  ElevatedButton(
                    onPressed: () => setState(() => showSignup = false),
                    child: const Text("Login"),
                  ),
                ],
              ),
            ],
          );

    return Scaffold(
      appBar: AppBar(title: const Text("Login")),
      body: content,
    );
  }
}
